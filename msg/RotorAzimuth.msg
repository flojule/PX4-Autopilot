# Coaxial rotor azimuth from the optical encoder (or SIH), drives the per-blade mixer
uint64 timestamp # time since system start (microseconds)

float32 azimuth # upper rotor blade 0 azimuth, wrapped to [0, 2 pi) (rad)
float32 speed   # upper rotor speed, positive (rad/s)
