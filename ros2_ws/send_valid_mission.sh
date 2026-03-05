#!/bin/bash

# 任务说明：
# 1. Pickup (取货点): Pad 1 (北向 20米) -> Lat: 47.3981508, Lon: 8.5461637
# 2. Dropoff (送货点): Pad 0 (原点/起飞点) -> Lat: 47.3979711, Lon: 8.5461637
#
# 这将强制无人机先飞往北方 20 米处取货，然后飞回原点送货。

echo "Sending mission: Pickup at Pad 1 (20m North), Dropoff at Origin..."

curl -X POST "http://localhost:8000/orders" \
     -H "Content-Type: application/json" \
     -d '{
       "pickup": {
         "lat": 47.3981508,
         "lon": 8.5461637,
         "alt": 0.5
       },
       "dropoff": {
         "lat": 47.3979711,
         "lon": 8.5461637,
         "alt": 0.5
       },
       "priority": "high",
       "deadline_ts": 1740000000
     }'

echo -e "\nMission sent!"
