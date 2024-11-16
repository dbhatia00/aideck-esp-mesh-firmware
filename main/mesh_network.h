#ifndef MESH_NETWORK_H
#define MESH_NETWORK_H

#include <stdint.h>
#include <stddef.h>

void mesh_network_init(void);
void mesh_send_data(const uint8_t *data, size_t len);

#endif // MESH_NETWORK_H
