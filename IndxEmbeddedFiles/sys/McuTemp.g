var interval = (exists(param.T)) ? param.T : 1
while true
  echo "Iteration", iterations, "MCU temp", boards[0].mcuTemp.current, "Board temp", sensors.analog[0].lastReading
  G4 S{var.interval}
