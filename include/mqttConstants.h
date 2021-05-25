#include "header.h"
#include "certificates.h"

//Ethernet Variable for connecting
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 117);
IPAddress myDns(192, 168, 1, 4);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);

// MQTT server IP and topic subscription
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
const char* pumps_primed = "hyd-1/self/pumps_primed";

//Last will and testament of disconnected Pod
const char* LWAT = "disconnected";

//SSL

// constant client cert
const char my_cert[] = "-----BEGIN CERTIFICATE-----\nMIIDcjCCAloCFAtx5U6P9/rDCwv6WzgLWS/yp/ZGMA0GCSqGSIb3DQEBCwUAMIGU\nMQswCQYDVQQGEwJQSDEPMA0GA1UECAwGTWFuaWxhMRIwEAYDVQQHDAlQYXJhbmFx\ndWUxFjAUBgNVBAoMDUhZRFJBLUNBLVJPT1QxEDAOBgNVBAsMB1JPT1QtQ0ExETAP\nBgNVBAMMCEhZRFJBLUNBMSMwIQYJKoZIhvcNAQkBFhRsZW5hcmQuZXhlQGdtYWls\nLmNvbTAeFw0yMTA1MTYwNjMzMTdaFw0yMjA1MTEwNjMzMTdaMFYxDjAMBgNVBAMM\nBUhZRC0xMQswCQYDVQQGEwJQSDEMMAoGA1UECAwDTUtUMRIwEAYDVQQHDAlQYXJh\nbmFxdWUxFTATBgNVBAoMDEhZRFJBLUNsaWVudDCCASIwDQYJKoZIhvcNAQEBBQAD\nggEPADCCAQoCggEBAMFvlqGqsCrv6tdeSdAX4AS9xfuJjD+ueH3wwy2doU4ZAxaQ\no6W74G37VKgVX0JVXYBNFeKV/YwRb9Gx+ZuU0phypDlAqffyyJkMzxIe57h1XN6l\nI9fE4eev0qWRw6cBW+fjrWFTZ56YUT8cGVREgiaqwZbM8zL/t5TOxgNOA+ltwVHm\nUYIs4eKsJRxByzIwjC8icFBFXOkL7Jr2heQB4A2S7kSuQ/Eus63pffU0Mc1gzm2Q\neOeBOQ0v0sdDtpR7IF0UnWmHSEvfaPpSBFwbTrSmndGPl3EeBZXL/4Xsq/qnPUpD\n0aB8oWaOPEq9RG53qmJM6hUm7DjoL9jn/UMmJbUCAwEAATANBgkqhkiG9w0BAQsF\nAAOCAQEAi9A0qMlTbrTN1R4V2rB6326CkIdTRj0cTE9z+q3cKNDrcI7T9Kb/G2K2\nG4f1u4PDDqxGxD9MnZeC3xmHyBiwgFDMxCsmNpPZFWoxu3zSPiqaDDVV7KNVSoM8\nh8JT8PuOp0IuP9bLyaei19+31df1S7HUMAuxrhafDm8BBrC4QaVzK6NpbNzo2VPP\ngbetyuk3f74sq2NsrBQ5UbEyZs5gwff7/ChcfxnIitU0vmk1AnIIc/I4i/iBfuQl\n0GdL1hNDg+WFvMEn4WkYAT06jZ9tGehlGGFDxy2ziqNoUGWu3FcdvkxggO5SJoNP\nK6HWz2dBTFATe16l4BRJdL2VAFCdUw==\n-----END CERTIFICATE-----\n";

const char my_key[] = "-----BEGIN RSA PRIVATE KEY-----\nMIIEowIBAAKCAQEAwW+WoaqwKu/q115J0BfgBL3F+4mMP654ffDDLZ2hThkDFpCj\npbvgbftUqBVfQlVdgE0V4pX9jBFv0bH5m5TSmHKkOUCp9/LImQzPEh7nuHVc3qUj\n18Th56/SpZHDpwFb5+OtYVNnnphRPxwZVESCJqrBlszzMv+3lM7GA04D6W3BUeZR\ngizh4qwlHEHLMjCMLyJwUEVc6QvsmvaF5AHgDZLuRK5D8S6zrel99TQxzWDObZB4\n54E5DS/Sx0O2lHsgXRSdaYdIS99o+lIEXBtOtKad0Y+XcR4Flcv/heyr+qc9SkPR\noHyhZo48Sr1EbneqYkzqFSbsOOgv2Of9QyYltQIDAQABAoIBAQCzd1iF/dscuyNj\nVfOdwcjyHTAGxAL/QlxAXJR5SfpSfxpCYUeziLAc2kYc/Fc5MAJj+yEG7KokvjjE\nkR9InbcWAackBz6q8PH0LdIgudO7bAgR+Z1bnysIzjPdsXOZCsW+S5qc5ckJd8BJ\nkURezoECZwLdaqFo+5/TzFQi1MsEUlcpicIOWclkJ9AdNRk93P/GZtSoTO490pYJ\nWqfKe/piilDdUcXGOFiS18xoGe/RLNPiPxxdXnG2vxiVDucoYVTMd6LnGclDfH/4\nBz+Z+z/ccd3Y9BGSVDbYRNHUvtOE9Aa13e6lxHDz0gl70DYeFKgq+SUwgs7/uqZW\n5C5Kqr2xAoGBAOS7MbpHdrKbwxcaJXHmP6Tr8yJXKdqlr0xOBletSLLBZ4VI1SGI\nvLNUKYB+wPx0IQjvv0gLHuRwFH/LwtdmfrHGP+JFJB/yt674Zo6AsexqQaO40998\nN1rTtih/jm9+CN+bH+h/iSafN4cCg/5A+8GRoN79sAy319H9JbomwtRfAoGBANh/\nMQfT6jUldYY7WSJu3B1btg2pNQMivMFTc+fudahgtm9eOOEZQAFqPFjgRNzAe1EX\nJE6ZnVQR3KlgSDe/3FhNDHo1ZNLGF5hfsZilON/GL3JUV6OEtzRDSnWPNx0P+rCR\nS5pMGgAXRTRw0frsJWeUMMQ8D1HgpYB1z0Z3k95rAoGAQQcXNCZtEIpIBKFiU+dp\n4LFUBkdRlt/T3pWwJ+0T5f4mODZkoYwGWdVrlyRyKVO6sJe4gfrnZXqX2V1Mn4B8\nNH5wWxvdEVgn8I6eWZUtAFGVJ768PaVK93+JIiphjEEAtNGdWLQpgdlyJ+gHLpm6\n+WU1ny/whS0fJowZ1bp/WXECgYBGxqYPyg6qGznljSdho15gVqgYpTWIdeXBSff9\nXa6nQh7CH8p2hFuxWrqpz3+Tr3VhGDCtZXjSkBhUYv6ZAycU0vN+TFBOu6XhEX+s\nyG7kcRT3ymzwLyX5dvoznoNFBAFPkeBma/SgDPt53lw3LNY5m2+p53yWpttUOK9v\neQ4lSwKBgGL4onDjVzlBxNmD2bLOVA2eNBIvFEPIgovi7wA8XZvNMPRn/tz0oUmk\nq8rJgDDhiseywOL2sGnRgAZSJGSZOUXVPANzuYGQIsfL9HTTXXNk1mzpqGWSivsP\nTYWWmCjpG3/MERexFlyp+VXCU5O71874gSLzhNprR5vv36JQQvUF\n-----END RSA PRIVATE KEY-----\n";

SSLClientParameters mTLS = SSLClientParameters::fromPEM(my_cert, sizeof my_cert, my_key, sizeof my_key);

// Ethernet and MQTT related objects
EthernetClient ethClient;
SSLClient ethClientSSL(ethClient, TAs, (size_t)TAs_NUM, A5);
PubSubClient mqttClient(server, 8883, ethClientSSL);