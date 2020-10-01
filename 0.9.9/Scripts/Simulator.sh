xrandr --output HDMI-0 --auto --mode 1920x1080 --pos 0x0 --rotate normal --primary
xrandr --output DP-1 --auto --mode 1920x1080 --pos 5760x0 --rotate normal
xrandr --output DP-5 --auto --mode 1920x1080 --pos 1920x0 --rotate normal
xrandr --output DP-3 --auto --mode 1920x1080 --pos 3840x0 --rotate normal

python manual_control_simulator.py --config display_config.yaml --display_name center &
python AdditionalDisplay.py --config display_config.yaml --display_name left --sleep=5 &
python AdditionalDisplay.py --config display_config.yaml --display_name right --sleep=5 &
python AdditionalDisplay.py --config display_config.yaml --display_name left_rear --sleep=5
