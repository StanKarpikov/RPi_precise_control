#!/bin/bash

timestamp_s() {
  echo $(date +"%s")
}

timestamp_us() {
  echo $(($(date +"%N")/1000))
}

timestamp_current(){
  echo "$(timestamp_s).$(timestamp_us)"
}

print_timestamp(){
  ((sec_part  = ($1 /1000)/1000000))
  ((usec_part = ($1 /1000)%1000000))
  echo "$sec_part.$usec_part"
}

timestamp_current_ns=
ts=
for i in {1..1}
do
    ((timestamp_current_ns = $(date +"%s")*1000000000 + $(date +"%N")))
    add_sec_open=5
    add_sec_close=15
    ((ts_open = timestamp_current_ns+$add_sec_open*1000000000))
    ((ts_close = ts_open+$add_sec_close*1000000000))
    ts=$(print_timestamp $ts_open)
    ts+=" "
    ts+=$(print_timestamp $ts_close)
    echo "Load timeout: $ts"
    echo $ts > /dev/ptime_control
    sleep 2
done
