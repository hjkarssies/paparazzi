# Outcome
2 flying bebops that avoid each other autonomously by means of ssid co-ordinate trick. 

-[ ] Must have 2 working bebops
-[ ] Fully calibrated on master (without my module, with full electronics)
-[ ] With antenna full electronics on the drone
-[ ] Module needs to work
-[ ] GPS reset NPS latest master rebase (MAG fix is here)
-[ ] ~Motive needs to work~

# changes
1 - [SOLVED] suspect communication problems due to (1) databuf and (2) esp_state being local.
2 - extern from vo to wifi is not there now.. instead extern from wifi to vo

# todo
-[.] calib mag both drones
-[ ] calib acc both drones
-[.] do something about old vel old head values...