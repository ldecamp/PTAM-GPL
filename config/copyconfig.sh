#!/bin/bash
echo "Copy calibrator settings"
cp calibrator_settings.cfg ../bin/calibrator_settings.cfg
echo "Merge global and hd cam settings"
rm ../bin/settings.cfg
# cat global_settings.cfg camSettings/webcam.cfg >> ../bin/settings.cfg
cat global_settings.cfg camSettings/hdcam.cfg >> ../bin/settings.cfg
echo "Config successfully deployed"