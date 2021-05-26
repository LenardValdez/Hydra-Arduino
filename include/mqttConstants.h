#include "header.h"
//this is autogenerated by SSClient python script
#include "certificates.h"

//Ethernet Variable for connecting
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 117);
IPAddress myDns(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT broker host/IP and topic subscription
// const char* server = "192.168.1.65";
const char* server = "mqtts.HYDRA-Server.com";
const char* topic_sensor_data = "hyd-1/sensor_data/";
const char* topic_probe_data = "hyd-1/probe_data/";
const char* command_new_crop = "hyd-1/commands/new_crop/";
const char* change_value_ph = "hyd-1/commands/ph_reading/";
const char* change_value_ec = "hyd-1/commands/ec_reading/";
const char* change_value_air_hum = "hyd-1/commands/air_humidity/";
const char* change_value_air_temp = "hyd-1/commands/air_temperature/";
const char* harvest_command = "hyd-1/commands/harvest/";
const char* connection = "hyd-1/connection/";
const char* manual_prime = "hyd-1/commands/manual_prime/";

//system self note
const char* pumps_primed = "hyd-1/self/pumps_primed/";
const char* EC_PH_time = "hyd-1/self/first_run/";

//Last will and testament of disconnected Pod
const char* LWAT = "disconnected";

//SSL section
// constant client and server/broker cert
const char my_cert[] = "-----BEGIN CERTIFICATE-----\n"
"MIIDcjCCAloCFAtx5U6P9/rDCwv6WzgLWS/yp/ZGMA0GCSqGSIb3DQEBCwUAMIGU\n"
"MQswCQYDVQQGEwJQSDEPMA0GA1UECAwGTWFuaWxhMRIwEAYDVQQHDAlQYXJhbmFx\n"
"dWUxFjAUBgNVBAoMDUhZRFJBLUNBLVJPT1QxEDAOBgNVBAsMB1JPT1QtQ0ExETAP\n"
"BgNVBAMMCEhZRFJBLUNBMSMwIQYJKoZIhvcNAQkBFhRsZW5hcmQuZXhlQGdtYWls\n"
"LmNvbTAeFw0yMTA1MTYwNjMzMTdaFw0yMjA1MTEwNjMzMTdaMFYxDjAMBgNVBAMM\n"
"BUhZRC0xMQswCQYDVQQGEwJQSDEMMAoGA1UECAwDTUtUMRIwEAYDVQQHDAlQYXJh\n"
"bmFxdWUxFTATBgNVBAoMDEhZRFJBLUNsaWVudDCCASIwDQYJKoZIhvcNAQEBBQAD\n"
"ggEPADCCAQoCggEBAMFvlqGqsCrv6tdeSdAX4AS9xfuJjD+ueH3wwy2doU4ZAxaQ\n"
"o6W74G37VKgVX0JVXYBNFeKV/YwRb9Gx+ZuU0phypDlAqffyyJkMzxIe57h1XN6l\n"
"I9fE4eev0qWRw6cBW+fjrWFTZ56YUT8cGVREgiaqwZbM8zL/t5TOxgNOA+ltwVHm\n"
"UYIs4eKsJRxByzIwjC8icFBFXOkL7Jr2heQB4A2S7kSuQ/Eus63pffU0Mc1gzm2Q\n"
"eOeBOQ0v0sdDtpR7IF0UnWmHSEvfaPpSBFwbTrSmndGPl3EeBZXL/4Xsq/qnPUpD\n"
"0aB8oWaOPEq9RG53qmJM6hUm7DjoL9jn/UMmJbUCAwEAATANBgkqhkiG9w0BAQsF\n"
"AAOCAQEAi9A0qMlTbrTN1R4V2rB6326CkIdTRj0cTE9z+q3cKNDrcI7T9Kb/G2K2\n"
"G4f1u4PDDqxGxD9MnZeC3xmHyBiwgFDMxCsmNpPZFWoxu3zSPiqaDDVV7KNVSoM8\n"
"h8JT8PuOp0IuP9bLyaei19+31df1S7HUMAuxrhafDm8BBrC4QaVzK6NpbNzo2VPP\n"
"gbetyuk3f74sq2NsrBQ5UbEyZs5gwff7/ChcfxnIitU0vmk1AnIIc/I4i/iBfuQl\n"
"0GdL1hNDg+WFvMEn4WkYAT06jZ9tGehlGGFDxy2ziqNoUGWu3FcdvkxggO5SJoNP\n"
"K6HWz2dBTFATe16l4BRJdL2VAFCdUw==\n"
"-----END CERTIFICATE-----\n";

const char my_key[] = "-----BEGIN RSA PRIVATE KEY-----\n"
"MIIEowIBAAKCAQEAwW+WoaqwKu/q115J0BfgBL3F+4mMP654ffDDLZ2hThkDFpCj\n"
"pbvgbftUqBVfQlVdgE0V4pX9jBFv0bH5m5TSmHKkOUCp9/LImQzPEh7nuHVc3qUj\n"
"18Th56/SpZHDpwFb5+OtYVNnnphRPxwZVESCJqrBlszzMv+3lM7GA04D6W3BUeZR\n"
"gizh4qwlHEHLMjCMLyJwUEVc6QvsmvaF5AHgDZLuRK5D8S6zrel99TQxzWDObZB4\n"
"54E5DS/Sx0O2lHsgXRSdaYdIS99o+lIEXBtOtKad0Y+XcR4Flcv/heyr+qc9SkPR\n"
"oHyhZo48Sr1EbneqYkzqFSbsOOgv2Of9QyYltQIDAQABAoIBAQCzd1iF/dscuyNj\n"
"VfOdwcjyHTAGxAL/QlxAXJR5SfpSfxpCYUeziLAc2kYc/Fc5MAJj+yEG7KokvjjE\n"
"kR9InbcWAackBz6q8PH0LdIgudO7bAgR+Z1bnysIzjPdsXOZCsW+S5qc5ckJd8BJ\n"
"kURezoECZwLdaqFo+5/TzFQi1MsEUlcpicIOWclkJ9AdNRk93P/GZtSoTO490pYJ\n"
"WqfKe/piilDdUcXGOFiS18xoGe/RLNPiPxxdXnG2vxiVDucoYVTMd6LnGclDfH/4\n"
"Bz+Z+z/ccd3Y9BGSVDbYRNHUvtOE9Aa13e6lxHDz0gl70DYeFKgq+SUwgs7/uqZW\n"
"5C5Kqr2xAoGBAOS7MbpHdrKbwxcaJXHmP6Tr8yJXKdqlr0xOBletSLLBZ4VI1SGI\n"
"vLNUKYB+wPx0IQjvv0gLHuRwFH/LwtdmfrHGP+JFJB/yt674Zo6AsexqQaO40998\n"
"N1rTtih/jm9+CN+bH+h/iSafN4cCg/5A+8GRoN79sAy319H9JbomwtRfAoGBANh/\n"
"MQfT6jUldYY7WSJu3B1btg2pNQMivMFTc+fudahgtm9eOOEZQAFqPFjgRNzAe1EX\n"
"JE6ZnVQR3KlgSDe/3FhNDHo1ZNLGF5hfsZilON/GL3JUV6OEtzRDSnWPNx0P+rCR\n"
"S5pMGgAXRTRw0frsJWeUMMQ8D1HgpYB1z0Z3k95rAoGAQQcXNCZtEIpIBKFiU+dp\n"
"4LFUBkdRlt/T3pWwJ+0T5f4mODZkoYwGWdVrlyRyKVO6sJe4gfrnZXqX2V1Mn4B8\n"
"NH5wWxvdEVgn8I6eWZUtAFGVJ768PaVK93+JIiphjEEAtNGdWLQpgdlyJ+gHLpm6\n"
"+WU1ny/whS0fJowZ1bp/WXECgYBGxqYPyg6qGznljSdho15gVqgYpTWIdeXBSff9\n"
"Xa6nQh7CH8p2hFuxWrqpz3+Tr3VhGDCtZXjSkBhUYv6ZAycU0vN+TFBOu6XhEX+s\n"
"yG7kcRT3ymzwLyX5dvoznoNFBAFPkeBma/SgDPt53lw3LNY5m2+p53yWpttUOK9v\n"
"eQ4lSwKBgGL4onDjVzlBxNmD2bLOVA2eNBIvFEPIgovi7wA8XZvNMPRn/tz0oUmk\n"
"q8rJgDDhiseywOL2sGnRgAZSJGSZOUXVPANzuYGQIsfL9HTTXXNk1mzpqGWSivsP\n"
"TYWWmCjpG3/MERexFlyp+VXCU5O71874gSLzhNprR5vv36JQQvUF\n"
"-----END RSA PRIVATE KEY-----\n";

SSLClientParameters mTLS = SSLClientParameters::fromPEM(my_cert, sizeof my_cert, my_key, sizeof my_key);

// Ethernet and MQTT related objects
EthernetClient ethClient;
SSLClient ethClientSSL(ethClient, TAs, (size_t)TAs_NUM, A5);
PubSubClient mqttClient(server, 8883, ethClientSSL);