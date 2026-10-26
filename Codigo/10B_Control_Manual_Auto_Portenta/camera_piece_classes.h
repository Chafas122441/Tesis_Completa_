/**
 * @file camera_piece_classes.h
 * @brief Normalizacion de etiquetas de las dos clases admitidas.
 * Descarta etiquetas ajenas antes de generar candidatos de recogida.
 */
#ifndef CAMERA_PIECE_CLASSES_H
#define CAMERA_PIECE_CLASSES_H

#include <stdint.h>
#include <string.h>

// IDs internos del DELTA, no IDs de aprendizaje de HUSKYLENS.
// El nombre es la unica autoridad. No reutilizar ID ni content como fallback.
static inline int16_t cameraPieceClassFromName(const char *name) {
  if (name == nullptr) return 0;
  while (*name == ' ' || *name == '\t' || *name == '\r' || *name == '\n') ++name;
  size_t length = strlen(name);
  while (length > 0u && (name[length - 1u] == ' ' || name[length - 1u] == '\t' ||
         name[length - 1u] == '\r' || name[length - 1u] == '\n')) --length;
  if (length == 7u && strncmp(name, "pieza_C", length) == 0) return 1;
  if (length == 11u && strncmp(name, "pieza_chica", length) == 0) return 2;
  return 0; // Ajena, desconocida o sin nombre. Nunca iniciar movimiento.
}

static inline const char *cameraPieceClassName(uint8_t id) {
  return id == 1u ? "pieza_C" : id == 2u ? "pieza_chica" : "NINGUNA";
}

#endif
