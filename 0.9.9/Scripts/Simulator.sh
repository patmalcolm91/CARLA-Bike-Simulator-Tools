# xrandr --output HDMI-0 --auto --mode 1920x1080 --pos 0x0 --rotate normal --primary
# xrandr --output DP-1 --auto --mode 1920x1080 --pos 5760x0 --rotate normal
# xrandr --output DP-5 --auto --mode 1920x1080 --pos 1920x0 --rotate normal
# xrandr --output DP-3 --auto --mode 1920x1080 --pos 3840x0 --rotate normal

python manual_control_simulator.py --config display_config_acs2.yaml --display_name center --scenario_config acs2_scenario_config.yaml &
python AdditionalDisplay.py --config display_config.yaml --display_name left --sleep=5 &
python AdditionalDisplay.py --config display_config.yaml --display_name right --sleep=5 &
python AdditionalDisplay.py --config display_config.yaml --display_name left_rear --sleep=5 &

cd /home/tum-vt/Desktop/carla-dev/Co-Simulation/Sumo
python run_synchronization.py --sumo-gui --sumo-port 8813 --sync-vehicle-lights /home/tum-vt/CARLA-Bike-Simulator-Tools/0.9.9/Scripts/AtCityStudy2/AtCity_Study2.sumocfg &

cd /home/tum-vt/CARLA-Bike-Simulator-Tools/0.9.9/Scripts &&
python TLSSync.py --net "/home/tum-vt/CARLA-Bike-Simulator-Tools/0.9.9/Scripts/AtCityStudy2/AtCity_Study2.net.xml" --config "acs2-tls.yaml" --run &

cd /home/tum-vt/CARLA-Bike-Simulator-Tools/0.9.9/Scripts/AtCityStudy2
python AtCity_Study2.py &
