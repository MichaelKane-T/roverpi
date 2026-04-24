#ifndef NET_H_
#define NET_H_

#include <stdint.h>

// This component is a wrapper around a lower-level communication method
// such as Wi-Fi. The complexities of establishing a network of devices and
// sending bytes through the network are abstracted away (to some degree).
// Peer addresses are 6 bytes in length (MAC address). The broadcast address
// is (0xff:0xff:0xff:0xff:0xff:0xff). A null pointer for the destination
// address will send to the group.

#define NET_ALEN 6 // Network address length in bytes
#define NET_MAX_WAIT (-1) // Wait forever


// Initialize the network.
// Return zero if successful, or non-zero otherwise.
int32_t net_init(void);

// Free resources used by the network.
// Return zero if successful, or non-zero otherwise.
int32_t net_deinit(void);

// Send data to the network.
// *dst: pointer to destination address (receiver). If NULL send to group.
// *buf: pointer to data buffer.
// size: size of data in bytes to write.
// wait: time to wait in ms for completion.
// Return number of bytes written, or negative number if error.
int32_t net_send(const uint8_t *dst, const void *buf, uint32_t size, uint32_t wait);

// Receive data from the network.
// *src: pointer to source address (sender).
// *buf: pointer to data buffer.
// size: size of buffer in bytes (maximum allowed to read).
// wait: time to wait in ms for completion.
// Return number of bytes read, or negative number if error.
int32_t net_recv(uint8_t *src, void *buf, uint32_t size, uint32_t wait);

// Open registration for a group, non blocking.
// id: id of the group.
// Return zero on success, or negative number if error.
int32_t net_group_open(uint32_t id);

// Close registration for a group, non blocking.
// Return zero on success, or negative number if error.
int32_t net_group_close(void);

// Clear registration for a group, non blocking.
// Return zero on success, or negative number if error.
int32_t net_group_clear(void);

// Return count of peers in the group, or negative number if error.
int32_t net_group_count(void);

#endif // NET_H_
