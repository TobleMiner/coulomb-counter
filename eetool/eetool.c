#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#include "../lipoCcounter/eeprom.h"
#include "../lipoCcounter/crc.h"
#include "../lipoCcounter/util.h"

#define LOG_OFFSET 0 //(sizeof(struct eeprom_device_block))
#define LOG_ENTRY_SIZE (sizeof(struct eeprom_log_block))
#define LOG_DATA_SIZE (sizeof(struct eeprom_log_data_priv))
#define NUM_BLOCKS (CLAMPH((EEPROM_SIZE - LOG_OFFSET) / LOG_ENTRY_SIZE, 255))

void hexdump(void* ptr, size_t len) {
  while(len--) {
    printf("%02x ", *((unsigned char*)ptr++));
  }
}

int validate_block(struct eeprom_log_block* block) {
  return block->crc == crc16_8((unsigned char*)&block->data, LOG_DATA_SIZE);
}

void parse_blocks(unsigned char* eeprom) {
  struct eeprom_log_block* blocks[NUM_BLOCKS], *newest_block = NULL, *last_block = NULL;
  int i, valid = 0, invalid = 0, newest_block_index, found = 0;
  last_block = (struct eeprom_log_block*)(eeprom + LOG_OFFSET + LOG_ENTRY_SIZE * (NUM_BLOCKS - 1));
  if(!validate_block(last_block)) {
    last_block = NULL;
  }
  for(i = 0; i < NUM_BLOCKS; i++) {
    struct eeprom_log_block* block = (struct eeprom_log_block*)(eeprom + LOG_OFFSET + LOG_ENTRY_SIZE * i);
    hexdump(block, sizeof(*block));
    blocks[i] = block;
    if(validate_block(block)) {
      valid++;
      if(last_block) {
        uint8_t serial = last_block->data.serial;
        serial++;
        if(serial != block->data.serial) {
          printf(" Sequence break");
          if(!found) {
            found = 1;
            newest_block_index = i - 1;
            newest_block = last_block;
          }
        }
      }
      if(!found) {
        newest_block_index = i;
        newest_block = block;
      }
      last_block = block;
    } else {
      printf(" INVALID");
      invalid++;
    }
    printf("\n");
  }
  printf("Got %ld blocks (size=%zu), %d are valid\n", NUM_BLOCKS, LOG_ENTRY_SIZE, valid);
  if(newest_block) {
    printf("Most recent valid block is at index %d:\n", newest_block_index);
    hexdump(newest_block, sizeof(*newest_block));
    printf("\n");
  }
}

int main(int argc, char** argv) {
  int err = 0, fd = 255;
  unsigned char eeprom[EEPROM_SIZE];
  size_t len = 0;

  if(argc < 2) {
    fprintf(stderr, "Usage: %s <eeprom file>\n", argv[0]);
    err = EINVAL;
    goto out;
  }

  if((fd = open(argv[1], O_RDONLY)) < 0) {
    err = errno;
    fprintf(stderr, "Failed to open '%s' for reading: %s (%d)\n", argv[1], strerror(err), err);
    goto out;
  }

  while(len < EEPROM_SIZE) {
    ssize_t read_len = read(fd, eeprom + len, EEPROM_SIZE - len);
    if(read_len < 0) {
      err = errno;
      fprintf(stderr, "Failed to read: %s (%d)\n", strerror(err), err);
      goto out_fd;
    }
    len += read_len;
  }

  parse_blocks(eeprom);
out_fd:
  close(fd);
out:
  return err;
}
