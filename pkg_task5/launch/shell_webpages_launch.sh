#!/bin/bash

# Store URLs in two variables
URL1="http://www.hivemq.com/demos/websocket-client/"
URL2="https://docs.google.com/spreadsheets/d/1phFlaOs974tuZCRpL_Lr2mcYeaHIc2INVvdcgVDByVE/edit?usp=sharing"

# Print some message
echo "** Opening $URL1 in Firefox **"
echo "** Opening $URL2 in Firefox **"
# Use firefox to open the two URLs in separate windows
firefox -new-window $URL1 $URL2



