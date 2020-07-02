
#define LILYGO_WATCH_2020_V1              // To use T-Watch2020 , please uncomment this line

// #include <TTGO.h>
#include <LilyGoWatch.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include <soc/rtc.h>
#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>


// Scan update time, 5 seems to be a good value.
#define SCAN_TIME_SECONDS 5

// When to forget old senders.
#define FORGET_AFTER_MINUTES 20

#define G_EVENT_VBUS_PLUGIN         _BV(0)
#define G_EVENT_VBUS_REMOVE         _BV(1)
#define G_EVENT_CHARGE_DONE         _BV(2)

#define G_EVENT_WIFI_SCAN_START     _BV(3)
#define G_EVENT_WIFI_SCAN_DONE      _BV(4)
#define G_EVENT_WIFI_CONNECTED      _BV(5)
#define G_EVENT_WIFI_BEGIN          _BV(6)
#define G_EVENT_WIFI_OFF            _BV(7)


#define WATCH_FLAG_SLEEP_MODE   _BV(1)
#define WATCH_FLAG_SLEEP_EXIT   _BV(2)
#define WATCH_FLAG_BMA_IRQ      _BV(3)
#define WATCH_FLAG_AXP_IRQ      _BV(4)


#define DEFAULT_SCREEN_TIMEOUT  30*1000

int counter = 0;
EventGroupHandle_t isr_group = NULL;
EventGroupHandle_t g_event_group = NULL;

QueueHandle_t g_event_queue_handle = NULL;
bool lenergy = false;
TTGOClass *ttgo;


BLEScan *scanner;
std::map<std::string, unsigned long> seenNotifiers;


/**
   Called when a new exposure notifier is seen.
*/
void onNewNotifierFound() {
  Serial.println("BEEP");
  counter = counter + 1;
  ttgo->tft->setTextColor(TFT_WHITE);
  ttgo->tft->fillRect(98, 100, 120, 40, TFT_BLACK);
  ttgo->tft->setCursor(80, 100);
  ttgo->tft->print("BCount:"); ttgo->tft->println(counter);

  //DO SHOW DISPLAY

}

enum {
  Q_EVENT_WIFI_SCAN_DONE,
  Q_EVENT_WIFI_CONNECT,
  Q_EVENT_BMA_INT,
  Q_EVENT_AXP_INT,
} ;

class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    /**
       Called when a BLE device is discovered.
    */
    void onResult(BLEAdvertisedDevice advertisedDevice) override {
      if (!advertisedDevice.haveServiceUUID()
          || !advertisedDevice.getServiceUUID().equals(BLEUUID((uint16_t) 0xfd6f))
         ) {
        return;
      }

      if (!seenNotifiers.count(advertisedDevice.getAddress().toString())) {
        // New notifier found.
        Serial.printf("+   %s \r\n", advertisedDevice.getAddress().toString().c_str());
        onNewNotifierFound();
      }
      else {
        // We've already seen this one.
        Serial.printf("... %s \r\n", advertisedDevice.getAddress().toString().c_str());
      }

      // Remember, with time.
      seenNotifiers[advertisedDevice.getAddress().toString()] = millis();
    }
};


/**
   Remove notifiers last seen over FORGET_AFTER_MINUTES ago.
*/
void forgetOldNotifiers() {
  for (auto const &notifier : seenNotifiers) {
    if (notifier.second + (FORGET_AFTER_MINUTES * 60 * 1000) < millis()) {
      Serial.printf("-   %s \r\n", notifier.first.c_str());
      seenNotifiers.erase(notifier.first);
    }
  }
}


void setup() {
  Serial.begin(115200);
  Serial.println("Hi.");
  isr_group = xEventGroupCreate();

  //Create a program that allows the required message objects and group flags
  g_event_queue_handle = xQueueCreate(20, sizeof(uint8_t));
  g_event_group = xEventGroupCreate();
  isr_group = xEventGroupCreate();

  ttgo = TTGOClass::getWatch();
  ttgo->begin();
  ttgo->power->adc1Enable(AXP202_BATT_VOL_ADC1 | AXP202_BATT_CUR_ADC1 | AXP202_VBUS_VOL_ADC1 | AXP202_VBUS_CUR_ADC1, AXP202_ON);
  ttgo->power->enableIRQ(AXP202_VBUS_REMOVED_IRQ | AXP202_VBUS_CONNECT_IRQ | AXP202_CHARGING_FINISHED_IRQ, AXP202_ON);
  ttgo->power->clearIRQ();

  // Turn off unused power
  ttgo->power->setPowerOutPut(AXP202_EXTEN, AXP202_OFF);
  ttgo->power->setPowerOutPut(AXP202_DCDC2, AXP202_OFF);
  ttgo->power->setPowerOutPut(AXP202_LDO3, AXP202_OFF);
  ttgo->power->setPowerOutPut(AXP202_LDO4, AXP202_OFF);

  ttgo->bma->begin();

  //Enable BMA423 interrupt
  ttgo->bma->attachInterrupt();

  // Connection interrupted to the specified pin
  pinMode(AXP202_INT, INPUT);
  attachInterrupt(AXP202_INT, [] {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    EventBits_t  bits = xEventGroupGetBitsFromISR(isr_group);
    if (bits & WATCH_FLAG_SLEEP_MODE)
    {
      //! For quick wake up, use the group flag
      xEventGroupSetBitsFromISR(isr_group, WATCH_FLAG_SLEEP_EXIT | WATCH_FLAG_AXP_IRQ, &xHigherPriorityTaskWoken);
    } else
    {
      uint8_t data = Q_EVENT_AXP_INT;
      xQueueSendFromISR(g_event_queue_handle, &data, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken)
    {
      portYIELD_FROM_ISR ();
    }
  }, FALLING);

  //Check if the RTC clock matches, if not, use compile time
  ttgo->rtc->check();

  //Synchronize time to system time
  ttgo->rtc->syncToSystem();



  ttgo->openBL();
  ttgo->tft->fillScreen(TFT_BLACK);
  ttgo->tft->setTextColor(random(0xFFFF));
  if (!ttgo->bma->begin()) {
    ttgo->tft->drawString("BMA423 Init FAIL",  25, 50, 4);
    while (1);
  }

  ttgo->tft->fillScreen(TFT_BLACK);
  ttgo->tft->drawString("Covid19 Beacons",  25, 50, 4);
  ttgo->tft->setTextFont(4);
  ttgo->tft->setTextColor(TFT_WHITE, TFT_BLACK);



  // Initialize scanner.
  BLEDevice::init("ESP");
  scanner = BLEDevice::getScan();
  scanner->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  scanner->setActiveScan(true); // Active scan uses more power, but gets results faster.
  scanner->setInterval(100);
  scanner->setWindow(99);

  Serial.println("Scanning ...");
}

void low_energy()
{
  if (ttgo->bl->isOn()) {
    xEventGroupSetBits(isr_group, WATCH_FLAG_SLEEP_MODE);
    ttgo->closeBL();



  } else {

    ttgo->openBL();
  }
}


void SetBatPercentage()
{

  int BatPet = ttgo->power->getBattPercentage();
  
  ttgo->tft->fillRect(98, 150, 70, 85, TFT_BLACK);
  ttgo->tft->setCursor(80, 150);

  if(BatPet > 25)
  {
    ttgo->tft->setTextColor(TFT_GREEN);
  }
  else
  {
    ttgo->tft->setTextColor(TFT_RED);
  }
  
  ttgo->tft->print("Bat: "); ttgo->tft->print(BatPet); ttgo->tft->println("%"); 

}

void loop() {
  bool  rlst;
  uint8_t data;
  static uint32_t start = 0;
  EventBits_t  bits = xEventGroupGetBits(isr_group);


  if (bits & WATCH_FLAG_SLEEP_EXIT) {
    if (lenergy) {
      lenergy = false;
      rtc_clk_cpu_freq_set(RTC_CPU_FREQ_160M);
    }

    low_energy();

    if (bits & WATCH_FLAG_BMA_IRQ) {
      do {
        rlst =  ttgo->bma->readInterrupt();
      } while (!rlst);
      xEventGroupClearBits(isr_group, WATCH_FLAG_BMA_IRQ);
    }
    if (bits & WATCH_FLAG_AXP_IRQ) {
      ttgo->power->readIRQ();
      ttgo->power->clearIRQ();
      //TODO: Only accept axp power pek key short press
      xEventGroupClearBits(isr_group, WATCH_FLAG_AXP_IRQ);
    }
    xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_EXIT);
    xEventGroupClearBits(isr_group, WATCH_FLAG_SLEEP_MODE);
  }
  if ((bits & WATCH_FLAG_SLEEP_MODE)) {
    //! No event processing after entering the information screen
    return;
  }

  if (xQueueReceive(g_event_queue_handle, &data, 5 / portTICK_RATE_MS) == pdPASS) {

    Serial.println(data);
    switch (data) {
      case Q_EVENT_AXP_INT:
        ttgo->power->readIRQ();
        if (ttgo->power->isPEKShortPressIRQ()) {
          ttgo->power->clearIRQ();

          low_energy();
          return;
        }
        ttgo->power->clearIRQ();
        break;

      default:
        break;
    }
  }
  
  Serial.println("-----");

  // Scan.
  scanner->start(SCAN_TIME_SECONDS, false);

  // Cleanup.
  scanner->clearResults();
  forgetOldNotifiers();

  Serial.printf("Count: %d \r\n", seenNotifiers.size());
  
  SetBatPercentage();
}
