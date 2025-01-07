#include <stdio.h>

int main() {
    FILE *file = fopen("input.bin", "wb");
    if (file != NULL) {
        unsigned char data[256] = {0};
        int data_size = 0;
        while (data_size < 256 && data[data_size] != 0) {
            data_size++;
        }

        // Compléter le reste du tableau avec 0xFF
        for (int i = data_size; i < 256; i++) {
            data[i] = 0xFF;
        }
        data[254] = {0x55};
        data[255] = {0xAA};
        fwrite(data, sizeof(data), 1, file);
        fclose(file);
    } else {
        printf("Error opening file.\n");
    }
    return 0;
}
