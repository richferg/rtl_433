/***
MRXSHR3 - Schrader Electronics SHR3
28103-FL000/28103-FL010
***/

#include "decoder.h"

static int tpms_subaru_decode(r_device *decoder, bitbuffer_t *bitbuffer, unsigned row __attribute__((unused)), unsigned bitpos __attribute__((unused)))
{
    // Copy bitbuffer to local var
    uint8_t b[9] = {bitbuffer->bb[0][3], bitbuffer->bb[0][4], bitbuffer->bb[0][5], bitbuffer->bb[0][6], bitbuffer->bb[0][7], bitbuffer->bb[0][8], bitbuffer->bb[0][9], bitbuffer->bb[0][10], '\0'};

    // Checksum - byte 7
    uint8_t sum    = 0;
    uint8_t chksum = b[7];
    for (uint8_t j = 0; j < 7; j++)
        sum += b[j];
    if (sum != chksum)
        return DECODE_FAIL_MIC;

    // Guesses
    uint8_t status = b[6];
    uint8_t temp = b[5];
    uint16_t pressure = (uint16_t)b[3] << 8 | b[4];

    // Sensor ID - bytes 0-2
    uint32_t id = (uint32_t)b[0] << 16 | b[1] << 8 | b[2];
    char id_str[7];
    snprintf(id_str, sizeof(id_str), "%06X", id);

    // Raw
    char raw[8 * 2 + 1]; // 8 bytes in hex notation
    snprintf(raw, sizeof(raw), "%02X%02X%02X%02X%02X%02X%02X%02X", b[0], b[1], b[2], b[3], b[4], b[5], b[6], b[7]);

    /*
    Information from 2019 Subaru Forester FSM for Australian Market

    Pressure information
    0 - 351.645kPa

    Status bit information
    7:   Low battery        Shows the remaining battery level						0: Normal
                                                                                    1: Low level
    6:   Sensor Fail        Shows the malfunction of TPMS							0: Normal
                                                                                    1: Malfunctioning
    5:   LF Response        Shows the responding transmission to the LF signal		0: Transmission for other than LF response
                                                                                    1: Transmission for LF response
    4:   PAL Condition      Shows the status of position information				0: Position detection finished
                                                                                    1: Position detection in progress
    3:   Rolling Detection  Shows the status of tire rotation detection				0: Tire stop status detected
                                                                                    1: Tire rotation status detected
    2-0: Status Code        Show the status in valve transmission					0: Responding to the LF signal "Learn LF"
                                                                                    1: Detecting the change in tire pressure
                                                                                    2: -
                                                                                    3: Responding to the LF signal "Entering off LF"
                                                                                    4: Position detection is in progress (transmission timing is synchronised with tire rotation)
                                                                                    5: Position detection is in progress (transmission timing is not synchronised with tire rotation)
                                                                                    6: Normal driving status (position detection mode is ended)
                                                                                    7: Detecting the completion of change in tire pressure
    */

    /* clang-format off */
    data_t *data = data_make(
            "model",            "",             DATA_STRING,    "Subaru",
            "type",             "",             DATA_STRING,    "TPMS",
            "id",               "",             DATA_STRING,    id_str,
            "status",           "",             DATA_FORMAT,    "0b%B", DATA_INT, status,
            "pressure_PSI",     "",             DATA_INT,       pressure,
            "temperature_C",    "",             DATA_INT,       temp-50, // Guessing for now. Seems pretty close.
            "raw",              "",             DATA_STRING,    raw,
            "mic",              "Integrity",    DATA_STRING,    "SUM8",
            NULL);
    /* clang-format on */

    decoder_output_data(decoder, data);
    return 1;
}

/** @sa tpms_subaru_decode() */
static int tpms_subaru_callback(r_device *decoder, bitbuffer_t *bitbuffer)
{
    uint8_t const preamble_pattern[3] = {0x7F, 0xFF, 0x00};

    unsigned bitpos = 0;
    int ret         = 0;
    int events      = 0;

    // Find a preamble with enough bits after it that it could be a complete packet
    while ((bitpos = bitbuffer_search(bitbuffer, 0, bitpos, preamble_pattern, 24)) + 89 <= bitbuffer->bits_per_row[0]) {
        ret = tpms_subaru_decode(decoder, bitbuffer, 0, bitpos + 23);
        if (ret > 0)
            events += ret;
        bitpos += 2;
    }

    return events > 0 ? events : ret;
}

static char const *const output_fields[] = {
        "model",
        "type",
        "id",
        "status",
        "pressure_PSI",
        "temperature_C",
        "raw",
        "mic",
        NULL,
};

r_device const tpms_subaru = {
        .name        = "Subaru TPMS",
        .modulation  = FSK_PULSE_MANCHESTER_ZEROBIT,
        .short_width = 50,
        .long_width  = 100,
        .reset_limit = 150,
        .decode_fn   = &tpms_subaru_callback,
        .fields      = output_fields,
};
