/*
 modified 12 Aug 2013
 by Soohwan Kim (suhwan@wiznet.co.kr)

- 10 Apr. 2015
 Added support for Arduino Ethernet Shield 2
 by Arduino.org team

 */

#include "Ethernet3.h"
#include "Dhcp.h"

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

// XXX: don't make assumptions about the value of MAX_SOCK_NUM.
uint8_t EthernetClass::_state[MAX_SOCK_NUM] = { 0, };
uint16_t EthernetClass::_server_port[MAX_SOCK_NUM] = { 0, };


int EthernetClass::begin(const uint8_t *mac_address, unsigned long timeout, unsigned long responseTimeout)
{
	if (_dhcp == nullptr)
	{
		_dhcp = new DhcpClass();
	}

	// Initialise the basic info
	w5500.init();
	w5500.setMACAddress(mac_address);
	w5500.setIPAddress(IPAddress(0,0,0,0).raw_address());

	// Now try to get our config info from a DHCP server
	const int ret = _dhcp->beginWithDHCP(mac_address, timeout, responseTimeout);
	if (ret == 1)
	{
		// We've successfully found a DHCP server and got our configuration info, so set things accordingly
		w5500.setIPAddress(_dhcp->getLocalIp().raw_address());
		w5500.setGatewayIp(_dhcp->getGatewayIp().raw_address());
		w5500.setSubnetMask(_dhcp->getSubnetMask().raw_address());
		_dnsServerAddress = _dhcp->getDnsServerIp();
	}

	return ret;
}

void EthernetClass::begin(const uint8_t *mac_address, IPAddress local_ip)
{
  IPAddress subnet(255, 255, 255, 0);
  begin(mac_address, local_ip, subnet);
}

void EthernetClass::begin(const uint8_t *mac_address, IPAddress local_ip, IPAddress subnet)
{
  // Assume the gateway will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress gateway = local_ip;
  gateway[3] = 1;
  begin(mac_address, local_ip, subnet, gateway);
}

void EthernetClass::begin(const uint8_t *mac_address, IPAddress local_ip, IPAddress subnet, IPAddress gateway)
{
  // Assume the DNS server will be the machine on the same network as the local IP
  // but with last octet being '1'
  IPAddress dns_server = local_ip;
  dns_server[3] = 1;
  begin(mac_address, local_ip, subnet, gateway, dns_server);
}

void EthernetClass::begin(const uint8_t *mac, IPAddress local_ip, IPAddress subnet, IPAddress gateway, IPAddress dns_server)
{
  w5500.init();
  w5500.setMACAddress(mac);
  w5500.setIPAddress(local_ip.raw_address());
  w5500.setGatewayIp(gateway.raw_address());
  w5500.setSubnetMask(subnet.raw_address());
  _dnsServerAddress = dns_server;
}

void EthernetClass::stop()
{
	w5500.stop();
}

int EthernetClass::maintain(){
  int rc = DHCP_CHECK_NONE;
  if(_dhcp != NULL){
    //we have a pointer to dhcp, use it
    rc = _dhcp->checkLease();
    switch ( rc ){
      case DHCP_CHECK_NONE:
        //nothing done
        break;
      case DHCP_CHECK_RENEW_OK:
      case DHCP_CHECK_REBIND_OK:
        //we might have got a new IP.
        w5500.setIPAddress(_dhcp->getLocalIp().raw_address());
        w5500.setGatewayIp(_dhcp->getGatewayIp().raw_address());
        w5500.setSubnetMask(_dhcp->getSubnetMask().raw_address());
        _dnsServerAddress = _dhcp->getDnsServerIp();
        break;
      default:
        //this is actually a error, it will retry though
        break;
    }
  }
  return rc;
}

uint8_t EthernetClass::phyState() {
  return w5500.getPHYCFGR();
  }

uint8_t EthernetClass::link() {
  return bitRead(w5500.getPHYCFGR(), 0);
  }

const char* EthernetClass::linkReport() {
  if(bitRead(w5500.getPHYCFGR(), 0) == 1) return "LINK";
  else return "NO LINK";
  }

uint8_t EthernetClass::speed() {
  if(bitRead(w5500.getPHYCFGR(), 0) == 1) {
    if(bitRead(w5500.getPHYCFGR(), 1) == 1) return 100;
    if(bitRead(w5500.getPHYCFGR(), 1) == 0) return 10;
    }
  return 0;
  }

const char* EthernetClass::speedReport() {
  if(bitRead(w5500.getPHYCFGR(), 0) == 1) {
    if(bitRead(w5500.getPHYCFGR(), 1) == 1) return "100 MB";
    if(bitRead(w5500.getPHYCFGR(), 1) == 0) return "10 MB";
    }
  return "NO LINK";
  }

uint8_t EthernetClass::duplex() {
  if(bitRead(w5500.getPHYCFGR(), 0) == 1) {
    if(bitRead(w5500.getPHYCFGR(), 2) == 1) return 2;
    if(bitRead(w5500.getPHYCFGR(), 2) == 0) return 1;
    }
  return 0;
  }

const char* EthernetClass::duplexReport() {
  if(bitRead(w5500.getPHYCFGR(), 0) == 1) {
    if(bitRead(w5500.getPHYCFGR(), 2) == 1) return "FULL DUPLEX";
    if(bitRead(w5500.getPHYCFGR(), 2) == 0) return "HALF DUPLEX";
    }
  return "NO LINK";
  }

void EthernetClass::macAddress(uint8_t mac[]) {
  w5500.getMACAddress(mac);
  }

const char* EthernetClass::macAddressReport() {
  uint8_t mac[6];
  static char str[18];
  w5500.getMACAddress(mac);
  sprintf(str, "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  return str;
  }

IPAddress EthernetClass::localIP()
{
  IPAddress ret;
  w5500.getIPAddress(ret.raw_address());
  return ret;
}

IPAddress EthernetClass::subnetMask()
{
  IPAddress ret;
  w5500.getSubnetMask(ret.raw_address());
  return ret;
}

IPAddress EthernetClass::gatewayIP()
{
  IPAddress ret;
  w5500.getGatewayIp(ret.raw_address());
  return ret;
}

IPAddress EthernetClass::dnsServerIP()
{
  return _dnsServerAddress;
}

EthernetClass Ethernet;
