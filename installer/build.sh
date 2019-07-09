#!/bin/bash

mkdir -p ROOT/tmp/RTI-RoR_X2/
cp "../RTI-RoR.ui" ROOT/tmp/RTI-RoR_X2/
cp "../RTI.png" ROOT/tmp/RTI-RoR_X2/
cp "../domelist RTI-RoR.txt" ROOT/tmp/RTI-RoR_X2/
cp "../build/Release/libRTI-RoR.dylib" ROOT/tmp/RTI-RoR_X2/

if [ ! -z "$installer_signature" ]; then
# signed package using env variable installer_signature
pkgbuild --root ROOT --identifier org.rti-zone.RTI-RoR_X2 --sign "$installer_signature" --scripts Scripts --version 1.0 RTI-RoR_X2.pkg
pkgutil --check-signature ./RTI-RoR_X2.pkg
else
pkgbuild --root ROOT --identifier org.rti-zone.RTI-RoR_X2 --scripts Scripts --version 1.0 RTI-RoR_X2.pkg
fi

rm -rf ROOT
