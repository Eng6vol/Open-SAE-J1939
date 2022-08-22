/*
 * Startup_ECU.c
 *
 *  Created on: 25 sep. 2021
 *      Author: Daniel Mårtensson
 */

#include "Open_SAE_J1939.h"
#include "string.h"

/* Layers */
#include "../Hardware/Hardware.h"
static bool HardCodeStruct(Information_this_ECU *information_this_ECU);

/* Load our ECU parameters into J1939 structure. Very useful if you want your ECU remember its NAME + address + identifications at startup. */
bool Open_SAE_J1939_Startup_ECU(J1939 *j1939) {
    uint32_t ECU_information_length = sizeof(Information_this_ECU);
    uint8_t ECU_information_data[ECU_information_length];
    memset(ECU_information_data, 0, ECU_information_length);
    // if (!Load_Struct(ECU_information_data, ECU_information_length, (char *)INFORMATION_THIS_ECU))
    //     return false; /* Problems occurs */
    HardCodeStruct((Information_this_ECU *) &ECU_information_data);
    memcpy(&j1939->information_this_ECU, (Information_this_ECU *)ECU_information_data, ECU_information_length);

    /* If we are going to send and receive the ECU identification and component identification, we need to specify the size of them */
    j1939->information_this_ECU.this_identifications.ecu_identification.length_of_each_field = 30;
    j1939->information_this_ECU.this_identifications.component_identification.length_of_each_field = 30;
    j1939->from_other_ecu_identifications.ecu_identification.length_of_each_field = 30;
    j1939->from_other_ecu_identifications.component_identification.length_of_each_field = 30;

    /* Clear other ECU addresses by setting the broadcast address to them */
    memset(j1939->other_ECU_address, 0xFF, 0xFF);
    j1939->number_of_cannot_claim_address = 0;
    j1939->number_of_other_ECU = 0;

    /* This broadcast out this ECU NAME + address to all other ECU:s */
    SAE_J1939_Response_Request_Address_Claimed(j1939);

    /* This asking all ECU about their NAME + address */
    SAE_J1939_Send_Request_Address_Claimed(j1939, 0xFF);

    /* OK */
    return true;
}

static bool HardCodeStruct(Information_this_ECU *information_this_ECU) {
    information_this_ECU->this_ECU_address = 0x51;
    /* This is the hard coded struct */
    information_this_ECU->this_name.identity_number = information_this_ECU->this_ECU_address;
    information_this_ECU->this_name.manufacturer_code = 10;
    information_this_ECU->this_name.function_instance = 0;
    information_this_ECU->this_name.ECU_instance = 0;
    information_this_ECU->this_name.function = 0;
    information_this_ECU->this_name.vehicle_system = 0;
    information_this_ECU->this_name.arbitrary_address_capable = 0;
    information_this_ECU->this_name.industry_group = 0;
    information_this_ECU->this_name.vehicle_system_instance = 0;
    information_this_ECU->this_name.from_ecu_address = information_this_ECU->this_ECU_address;


    information_this_ECU->this_identifications.software_identification.number_of_fields = 6;
    memcpy(information_this_ECU->this_identifications.software_identification.identifications, "v1.0", sizeof("v1.0"));
    information_this_ECU->this_identifications.software_identification.from_ecu_address = information_this_ECU->this_ECU_address;

    information_this_ECU->this_identifications.component_identification.length_of_each_field = 10;
    memcpy(information_this_ECU->this_identifications.component_identification.component_product_date, "01/01/2020", sizeof("01/01/2020"));
    memcpy(information_this_ECU->this_identifications.component_identification.component_model_name, "0", sizeof("0"));
    memcpy(information_this_ECU->this_identifications.component_identification.component_serial_number, "001", sizeof("001"));
    memcpy(information_this_ECU->this_identifications.component_identification.component_unit_name, "001", sizeof("001"));
    information_this_ECU->this_identifications.component_identification.from_ecu_address = information_this_ECU->this_ECU_address;

    information_this_ECU->this_identifications.ecu_identification.length_of_each_field = 30;
	char ecu_part_number[20] = "ABC-1100P-XXXX10";
	char ecu_serial_number[20] = "1-200-K-10M";
	char ecu_location[20] = "Under bridge";
	char ecu_type[20] = "Model G";
	for(uint8_t i = 0; i < 20; i++){
		information_this_ECU->this_identifications.ecu_identification.ecu_part_number[i] = (uint8_t) ecu_part_number[i];
		information_this_ECU->this_identifications.ecu_identification.ecu_serial_number[i] = (uint8_t) ecu_serial_number[i];
		information_this_ECU->this_identifications.ecu_identification.ecu_location[i] = (uint8_t) ecu_location[i];
		information_this_ECU->this_identifications.ecu_identification.ecu_type[i] = (uint8_t) ecu_type[i];
	}
    return true;
}
