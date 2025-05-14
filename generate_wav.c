#include <stdio.h>
#include <stdint.h>
#include <string.h>

struct wav_header
{
    char riff[4];
    int32_t fSize;
    char wave[4];
    char fmt[4];
    int32_t chunkSize;
    int16_t formatTag;
    int16_t channels;
    int32_t sampleRate;
    int32_t byteRate;
    int16_t bytesPerSample;
    int16_t bitPerSample;
    char data[4];
    int32_t dataLength;
};

int main(int argc, char *argv[])
{
    const int sampleRate = 6400; // 6400 samples per second
    struct wav_header header;

    // Set up WAV header fields
    strncpy(header.riff, "RIFF", 4);
    strncpy(header.wave, "WAVE", 4);
    strncpy(header.fmt, "fmt ", 4);
    strncpy(header.data, "data", 4);

    header.chunkSize = 16;
    header.formatTag = 1;       // PCM
    header.channels = 1;        // Mono
    header.sampleRate = sampleRate; // 6400 samples per second
    header.bitPerSample = 16;  // 16 bits per sample
    header.bytesPerSample = (header.bitPerSample / 8) * header.channels; // 2 bytes per sample
    header.byteRate = header.sampleRate * header.bytesPerSample;

    // const int duration = 10;  // 10 seconds
    if (argc < 2) {
        printf("Usage: %s <duration_in_seconds>\n", argv[0]);
        return 1;
    }
    int duration = atoi(argv[1]); // Duration in seconds
    if (duration <= 0) {
        printf("Invalid duration. Please provide a positive integer.\n");
        return 1;
    }

    const int bufferSize = sampleRate * duration; // Number of samples
    header.dataLength = bufferSize * header.bytesPerSample;
    header.fSize = 44 + header.dataLength; // 44 bytes header + audio data length

    short int buffer[bufferSize]; // Audio data buffer

    // Open the raw ADC data file
    FILE *fp = fopen("raw_adc_values.txt", "rb");
    if (fp == NULL)
    {
        perror("Error opening file");
        return 1;
    }

    for (int i = 0; i < bufferSize; i++)
    {
        uint16_t adcVal;
        fread(&adcVal, sizeof(uint16_t), 1, fp);  // Read a 2-byte ADC value

        // Scale ADC value from [0, 4095] to signed 16-bit range [-32768, 32767]
        buffer[i] = (short int)(((int)adcVal - 2048) * 16); // Center around 0
    }
    // Close the raw ADC data file
    fclose(fp);

    // Open the WAV file for writing
    FILE *wavFile = fopen("output.wav", "wb");
    if (wavFile == NULL)
    {
        perror("Error creating WAV file");
        return 1;
    }

    // Write the WAV header
    fwrite(&header, sizeof(header), 1, wavFile);

    // Write the audio data (scaled ADC values)
    fwrite(buffer, sizeof(short int), bufferSize, wavFile);

    // Close the WAV file
    fclose(wavFile);

    return 0;
}
