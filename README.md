# Digital electronics presentation - code samples
Sketches for using a 23LC1024 SRAM chip with an Arduino Uno  


The sram_basic sketch contains implementations for writing simple c data types to the 23LC1024 SRAM chip.

The weather sktech makes use of the functions implemented in sram_basic for creating a "weather station".
It makes use of a DHT22 sensor and the DS3231 RTC module in order to save temperature and humidity measurements to the SRAM with the corresponding unix timestamp.
