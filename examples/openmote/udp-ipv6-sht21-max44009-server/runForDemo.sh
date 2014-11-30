#!/bin/bash

while true;
do
  echo "light" | nc6 --udp  aaaa::212:4b00:433:edae 3000 -q 1
  echo "  <== reading light information (in terms of lux)"
  sleep 2
  echo "humidity" | nc6 --udp  aaaa::212:4b00:433:edae 3000 -q 1
  echo "  <== reading humidity information (in terms of %)"
  sleep 6
done
