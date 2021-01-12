#!/bin/bash

# Store URLs in two variables
URL1="http://www.hivemq.com/demos/websocket-client/"


# Print some message
echo "** Opening $URL1 in Firefox **"

# Use firefox to open the two URLs in separate windows
firefox -new-window $URL1
