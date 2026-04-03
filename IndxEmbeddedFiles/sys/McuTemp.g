while true
  echo "MCU:", boards[0].mcuTemp.current, ", Board:", sensors.analog[0].lastReading
  G4 S5
