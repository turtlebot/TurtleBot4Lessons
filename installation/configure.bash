
#!/bin/bash
echo "::::::::::::::::::::::Configuring TurtleBot4 Network Interfaces::::::::::::::::"
echo "::::::::::::::::::::::Navigating to /Home Directory::::::::::::::::::::::::::::"

# Navigating to the home directory
cd $HOME

echo "::::::::::::::::::::::Creating cyclonedds_pc.xml:::::::::::::::::::::::::::::::"
# creating a xml file for configuring cyclone DDS

exec 3>cyclonedds_pc.xml #Creating xml file

#HEADER --- Template for XML file
echo '<CycloneDDS>' >&3
echo '    <Domain>' >&3
echo "        <General>" >&3
echo '            <DontRoute>true</DontRoute>' >&3
echo '            <NetworkInterfaceAddress>wlp0s20f3</NetworkInterfaceAddress>' >&3
echo '        </General>' >&3
echo '    </Domain>' >&3
echo '</CycloneDDS>' >&3

FILE=/home/"$USER"/cyclonedds_pc.xml

# Verifying if the xml file exists
if test -f "$FILE"; then
    echo "$FILE was created and exists."
    echo ":::::::::::::::::::::Succesfully Created file:::::::::::::::::::::::::::::::"
else
    echo "Error creating XML file"
fi

echo ":::::::::::::::::::::Moving the xml file to /etc/::::::::::::::::::::::::::"
cd $HOME
# Move the xml file to a convenient location:
sudo mv cyclonedds_pc.xml /etc/

echo ":::::::::::::::::::::Adding config to bashrc::::::::::::::::::::::::::::::"
# Add this line to your ~/.bashrc file to automatically configure CycloneDDS each time you open a new terminal
export CYCLONEDDS_URI=/etc/cyclonedds_pc.xml

echo ":::::::::::::::::::::Sourcing bashrc::::::::::::::::::::::::::::::::::::::"
# Source ~/.bashrc to apply the configurations to your current terminal
source ~/.bashrc

# echo "::::::::::::::::::::::::::::::Configuring WIFI::::::::::::::::::::::::::::::"

# sudo apt-get install network-manager

# sudo systemctl start NetworkManager.service 
# sudo systemctl enable NetworkManager.service

# echo "::::::::::::::::::::::::::::::Connecting to Turtlebot4::::::::::::::::::::::::::::::"
# nmcli dev wifi connect Turtlebot4 password Turtlebot4

# connection_response=$(nmcli dev wifi connect Turtlebot4 password Turtlebot4)

# if [[ $connection_response =~ "successfully" ]]; then
#   echo "Connection successfully to Turtlebot4 WiFi"

# else

#   echo "Reconnecting to TurtleBot4 WiFi"
#   connection_response=$(nmcli dev wifi connect Turtlebot4 password Turtlebot4)
#     x=1
#     while [ $x -le 5 ]
#     do
#         echo "::Atempt $x::"
#         nmcli dev wifi connect Turtlebot4 password Turtlebot4
#         connection_response=$(nmcli dev wifi connect Turtlebot4 password Turtlebot4)
#         if [[ $connection_response =~ "Error" ]]; then
#             echo "::Attempt Failed::"
#         else 
#             echo "Connection established and Attempt Successfull"
#             exit
        
#         x=$(( $x + 1 ))
#     done

#    else
#         echo "Connection Failed to Turtlebot4 WiFi..." 
#         echo "Connect Mannually to the Turtlebot4 network" 
#    fi 
    
# fi


# echo ":::::::::::::::::::::::::Establishing connection to Turtlebot4 Rasperberry::::::::::::::::::::"
# ssh ubuntu@10.42.0.1