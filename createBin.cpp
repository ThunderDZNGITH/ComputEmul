#include <stdio.h>

int main() {
    FILE *file = fopen("input.bin", "wb");
    if (file != NULL) {
        unsigned char data[] = {0x05, 0b11001000, 0x05, 0b11010000, 0x01, 0b11001010, 0x06, 0b11010000, 0x0B, 0b11100001, 0x0B, 0b11010001, 0x0B, 0b11001000};
        fwrite(data, sizeof(data), 1, file);
        fclose(file);
    } else {
        printf("Error opening file.\n");
    }
    return 0;
}
