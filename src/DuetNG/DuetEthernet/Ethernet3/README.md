# Ethernet3
Ethernet library for Arduino and Ethernetshield2 / WIZ550io / WIZ850io with Wiznet W5500 chip
based on the Ethernet2 library of arduino.org

added some new functionalities

you need to include

    #include <Ethernet3.h> // instead Ethernet.h
    #include <EthernetUdp3.h> // instead EthernetUdp.h for UDP functionality
    
###- New init procedure **!!!**

the init of the Ethernetinterface changed, the ordner is now:

*mac, ip, subnet, gateway, dns* instead *mac, ip, dns, gateway, subnet*,
which is more logical

    Ethernet.begin(mac, ip, subnet, gateway, dns);
    
###- Multicast support

multicast for udp added

    EthernetUdp udp
    upd.beginMulticast(multicastIP, port);

###- PHY support

added some function to read the PHYCFGR in Ethernet3

    uint8_t phyState(); // returns the PHYCFGR
    uint8_t link(); // returns the linkstate, 1 = linked, 0 = no link
    const char* linkReport(); // returns the linkstate as a string
    uint8_t speed(); // returns speed in MB/s
    const char* speedReport(); // returns speed as a string
    uint8_t duplex(); // returns duplex mode 0 = no link, 1 = Half Duplex, 2 = Full Duplex
    const char* duplexReport(); // returns duplex mode as a string

example

    Serial.println(Ethernet.linkReport()); 

###- MAC address

added some function to read the MAC address in Ethernet3, this is helpfull when you use Wiznet boards like WIZ550io with build in MAC address

    void macAddress(uint8_t mac[]); // get the MAC Address
    const char* macAddressReport(); // returns the the MAC Address as a string

example

    uint8_t mac[6]; // array for mac address
    Ethernet.macAddress(mac);
