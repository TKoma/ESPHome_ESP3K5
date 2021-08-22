#include "esphome.h"

// ESP_LOGE - error (lowest)
// ESP_LOGW - warning
// ESP_LOGI - info
// ESP_LOGD - debug
// ESP_LOGV - verbose (highest)

// PC -> Inverter (9600bps)
// byte0	byte1	byte2	byte3	byte4	byte5	byte6
// Header1	Header2	ID		Cmd		Length	Tail	Checksum
// 0x0A		0x96	0x01*	0x54	0x18	0x05	0x6D**
// * ID : Station ID. (last two digit of serial number)
// ** Checksum : sum of byte2~4

// Return Packet
// 0.Header1				0xB1
// 1.Header2				0xB7
// 2.StationID				0x01 -
// 3.SolarVoltage1_LSB				(/10) "v"
// 4.SolarVoltage1_MSG
// 5.SolarCurrent_LSB				(/10) "A"
// 6.SolarCurrent_MSB
// 7.SolarVoltage2_LSB				(/10) "v"
// 8.SolarVoltage2_MSB
// 9.LineVoltage_LSB				(/10) "v"
// 10.LineVoltage_MSB
// 11.LineCurrent_LSB				(/10) "A"
// 12.LineCurrent_MSB
// 13.Temperature_LSB				(/10) "°C"
// 14.Temperature_MSB
// 15.EnergyToday_LSB				(/100) "kWh"
// 16.EnergyToday_MSB
// 17.EnergyTotal_byte0				"kWh"
// 18.EnergyTotal_byte1
// 19.EnergyTotal_byte2
// 20.FaultCode_byte0
// 21.FaultCode_byte1
// 22.FaultCode_byte2
// 23.FaultCode_byte3
// 24.RunStatus				0x01	(1:RUN, 0:STOP)
// 25.Frequency_LSB					(/10) "Hz"
// 26.Frequency_MSB
// 27.OperationTime_LSB				"min"
// 28.OperationTime_MSB
// 29.PowerFactor					(/100)
// 30.DSPVersion					(/100) "Ver"
// 31.Checksum						(XOR=(0:30))

// Fault Code Table
// 0x00000001						Solar OverCurrent
// 0x00000002						Solar OverVoltage
// 0x00000004						Solar LowVoltage
// 0x00000008						DCLink OverVoltage
// 0x00000010						DCLink LowVoltage
// 0x00000020						Inverter OverCurrent
// 0x00000040						Line OverVoltage
// 0x00000080						Line LowVoltage
// 0x00000100						OverTemperature
// 0x00000200						Line High Frequency
// 0x00000400						Line Low Frequency
// 0x00000800						Solar OverPower
// 0x00001000						DC 
// 0x00002000						DC Leakage
// 0x00010000						Drive Alone
// 0x00020000						Inverter OverCurrent HW

// test return packet
// 0xB1 0xB7 0x01 0x15 0x0E 0x05 0x01 0x01 0x0E 0x9A 0x08 0xC8 0x01 0x60 0x01 0xFF 0x00 0x9F 0x86 0x01 0x00 0x00 0x00 0x00 0x01 0x59 0x02 0x01 0x01 0x63 0x6E 0xC1

static const uint8_t ESP3K5_CMD_REQUEST[] = { 0x0A, 0x96, 0x61/*97*/, 0x54, 0x18, 0x05 };		// + (checksum)0xCD

static const char *TAG = "custom_esp3k5";

class custom_esp3k5 : public PollingComponent, public UARTDevice, public Sensor {
	public:
		custom_esp3k5(UARTComponent *parent, unsigned int interval) : PollingComponent(interval), UARTDevice(parent) {}

		Sensor *espUptime = new Sensor();
		
		Sensor *solarVoltage1 = new Sensor();
		Sensor *solarCurrent = new Sensor();
		Sensor *solarVoltage2 = new Sensor();
		Sensor *lineVoltage = new Sensor();
		Sensor *lineCurrent = new Sensor();
		Sensor *temperature = new Sensor();
		Sensor *energyToday = new Sensor();
		Sensor *energyTotal = new Sensor();
		Sensor *faultCode = new Sensor();
		Sensor *runStatus = new Sensor();
		Sensor *frequency = new Sensor();
		Sensor *operationTime = new Sensor();
		Sensor *powerFactor = new Sensor();
		Sensor *dspVersion = new Sensor();
		
		uint8_t recv[32];
		uint8_t recvIndex;
		
		

		float get_setup_priority() const override { return esphome::setup_priority::DATA; }

		void setup() override {

			ESP_LOGD(TAG, "ESP3K5 setup");
			
			recvIndex = 0;
			
		}

		void update() override {
			this->write_array(ESP3K5_CMD_REQUEST, 6);
			this->write_byte(cmd_checksum(ESP3K5_CMD_REQUEST));
			this->flush();

			if (espUptime != nullptr)
				espUptime->publish_state(millis() / 60000);
		}
		
		void loop() override {
			
			while (this->available()) {
				recv[recvIndex++] = this->read();
				
				if (recvIndex == 1) {
					if (recv[0] != 0xB1) {
						recvIndex = 0;
						ESP_LOGD(TAG, "received header1 error");
					}
				}
				else if (recvIndex == 2) {
					if (recv[1] != 0xB7) {
						recvIndex = 0;
						ESP_LOGD(TAG, "received header2 error");
					}
				}
				else if (recvIndex >= 32) {
					recvIndex = 0;
					
					if (recv_checksum(recv)) {
						ESP_LOGD(TAG, "received valid packet");
						
						uint32_t temp;
						
						// solarVoltage1	3:4
						if (solarVoltage1 != nullptr)
							solarVoltage1->publish_state(b2f(recv[3], recv[4]) / 10);
						
						// solarCurrent		5:6
						if (solarCurrent != nullptr)
							solarCurrent->publish_state(b2f(recv[5], recv[6]) / 10);
						
						// solarVoltage2	7:8
						if (solarVoltage2 != nullptr)
							solarVoltage2->publish_state(b2f(recv[7], recv[8]) / 10);
						
						// lineVoltage		9:10
						if (lineVoltage != nullptr)
							lineVoltage->publish_state(b2f(recv[9], recv[10]) / 10);
						
						// lineCurrent		11:12
						if (lineCurrent != nullptr)
							lineCurrent->publish_state(b2f(recv[11], recv[12]) / 10);
						
						// temperature		13:14
						if (temperature != nullptr)
							temperature->publish_state(b2f(recv[13], recv[14]) / 10);
						
						// energyToday		15:16
						if (energyToday != nullptr)
							energyToday->publish_state(b2f(recv[15], recv[16]) / 100);
						
						// energyTotal		17:19
						temp = recv[19];
						temp = temp * 256 + recv[18];
						temp = temp * 256 + recv[17];
						if (energyTotal != nullptr)
							energyTotal->publish_state(temp);
						
						// faultCode		20:23
						temp = recv[23];
						temp = temp * 256 + recv[22];
						temp = temp * 256 + recv[21];
						temp = temp * 256 + recv[20];
						if (faultCode != nullptr)
							faultCode->publish_state(temp);
						
						// runStatus		24
						if (runStatus != nullptr)
							runStatus->publish_state(recv[24]);
						
						// frequency		25:26
						if (frequency != nullptr)
							frequency->publish_state(b2f(recv[25], recv[26]) / 10);
						
						// operationTime	27:28
						temp = recv[28];
						temp = temp * 256 + recv[27];
						if (operationTime != nullptr)
							operationTime->publish_state(temp);
						
						// powerFactor		29
						if (powerFactor != nullptr)
							powerFactor->publish_state((float)recv[29] / 100);
						
						// dspVersion		30
						if (dspVersion != nullptr)
							dspVersion->publish_state((float)recv[30] / 100);
						
					}
					else {
						ESP_LOGD(TAG, "received packet checksum error");
					}
				}
			}
		}
		
		uint8_t cmd_checksum(const uint8_t *packet) {
			uint8_t sum = 0;
			for (uint8_t i = 2; i < 5; i++)
				sum += packet[i];
			return sum;
		}
		
		bool recv_checksum(uint8_t *packet) {
			uint8_t sum = 0;
			for (uint8_t i = 0; i < 31; i++)
				sum = sum ^ packet[i];
			return (packet[31] == sum);
		}
		
		float b2f(uint8_t lsb, uint8_t msb)
		{
			uint16_t temp;
			
			temp = msb;
			temp = temp * 256 + lsb;
			return (float)temp;
		}
};
