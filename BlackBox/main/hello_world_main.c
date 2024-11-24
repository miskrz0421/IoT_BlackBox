#include <stdio.h>
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include "esp_partition.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_chip_info.h"
#include "esp_psram.h"
#include "esp_heap_caps.h"
#include "spi_flash_chip_driver.h"

void app_main(void)
{
    // Informacje o pamięci Flash
    uint32_t flash_size;
    esp_flash_get_size(NULL, &flash_size);
    
    printf("\n=== Informacje o pamięci ESP32-S3 ===\n");
    printf("Rozmiar pamięci Flash: %ld MB\n", flash_size / (1024 * 1024));
    
    // Informacje o PSRAM używając heap_caps
    size_t spiram_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    if(spiram_size > 0) {
        size_t free_spiram = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
        size_t largest_spiram_block = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
        
        printf("\nInformacje o PSRAM:\n");
        printf("Całkowity rozmiar PSRAM: %d KB\n", spiram_size / 1024);
        printf("Wolna pamięć PSRAM: %d KB\n", free_spiram / 1024);
        printf("Największy wolny blok PSRAM: %d KB\n", largest_spiram_block / 1024);
    } else {
        printf("\nPSRAM nie jest dostępny lub nie jest włączony\n");
    }
    
    // Informacje o partycjach
    esp_partition_iterator_t it = esp_partition_find(ESP_PARTITION_TYPE_ANY, ESP_PARTITION_SUBTYPE_ANY, NULL);
    printf("\nLista partycji:\n");
    printf("Typ  | Podtyp | Adres    | Rozmiar  | Etykieta\n");
    printf("-----+--------+----------+----------+----------\n");
    
    while (it != NULL) {
        const esp_partition_t *partition = esp_partition_get(it);
        printf("0x%02x | 0x%02x   | 0x%06lx | %6ldKB | %s\n",
               partition->type,
               partition->subtype,
               partition->address,
               partition->size / 1024,
               partition->label);
        it = esp_partition_next(it);
    }
    esp_partition_iterator_release(it);
    
    // Informacje o pamięci RAM
    printf("\nInformacje o pamięci RAM:\n");
    printf("Heap wolna pamięć: %ld bytes\n", esp_get_free_heap_size());
    printf("Najmniejszy blok heap: %ld bytes\n", esp_get_minimum_free_heap_size());
    
    // Szczegółowe informacje o różnych typach pamięci
    printf("\nSzczegółowe informacje o pamięci:\n");
    printf("DRAM wolna: %d KB\n", heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024);
    printf("DRAM największy wolny blok: %d KB\n", heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024);
    printf("Wolna pamięć DMA: %d KB\n", heap_caps_get_free_size(MALLOC_CAP_DMA) / 1024);
    printf("Największy wolny blok DMA: %d KB\n", heap_caps_get_largest_free_block(MALLOC_CAP_DMA) / 1024);
    
    // Informacje o chip
    esp_chip_info_t chip_info;
    esp_chip_info(&chip_info);
    
    printf("\nInformacje o chip:\n");
    printf("Model: ESP32-S3\n");
    printf("Liczba rdzeni: %d\n", chip_info.cores);
    printf("Wersja: %d\n", chip_info.revision);
    printf("Funkcje: %s%s%s%s\n",
           (chip_info.features & CHIP_FEATURE_WIFI_BGN) ? "WiFi " : "",
           (chip_info.features & CHIP_FEATURE_BLE) ? "BLE " : "",
           (chip_info.features & CHIP_FEATURE_IEEE802154) ? "IEEE 802.15.4 " : "",
           (chip_info.features & CHIP_FEATURE_EMB_FLASH) ? "Embedded Flash " : "");
}