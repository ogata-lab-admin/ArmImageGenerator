#!/bin/bash
RTD=/localhost/vega.local.host_cxt/
rtact ${RTD}MikataArm0.rtc
rtcon ${RTD}MikataArm0.rtc:manipMiddle ${RTD}ArmImageGenerator0.rtc:manipMiddle
rtcon ${RTD}MikataArm0.rtc:manipCommon ${RTD}ArmImageGenerator0.rtc:manipCommon

sleep 1

rtact ${RTD}ArmImageGenerator0.rtc