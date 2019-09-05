#include "MLX90615.h"
#include <I2cMaster.h>

MLX90615 mlx90615(DEVICE_ADDR, &Wire);

void setup()
{
  Serial.begin(9600);
  Serial.println("Setup...");
  Wire.begin();
  //mlx90615.writeEmissivity(Default_Emissivity); //write data into EEPROM when you need to adjust emissivity.
  //mlx90615.readEEPROM(); //read EEPROM data to check whether it's a default one.
}

void loop()
{
  Serial.print("Object temperature: ");
  Serial.println(mlx90615.getTemperature(MLX90615_OBJECT_TEMPERATURE));
  Serial.print("Ambient temperature: ");
  Serial.println(mlx90615.getTemperature(MLX90615_AMBIENT_TEMPERATURE));
  
  delay(1000);
}
