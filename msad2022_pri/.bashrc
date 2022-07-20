#--- begin autostart feature for HackSPi/RasPike -------
if [ -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.2" ]; then
    mv -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.2" "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.3"
fi
if [ -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.1" ]; then
    mv -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.1" "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.2"
fi
if [ -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log" ]; then
    mv -f "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log" "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log.1"
fi
"/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/hackspi" | tee "/home/shojiro-2/work/RasPike/sdk/workspace/msad2022_pri/.hackspi/autostart.log"
#--------- autostart feature for HackSPi/RasPike end ---
