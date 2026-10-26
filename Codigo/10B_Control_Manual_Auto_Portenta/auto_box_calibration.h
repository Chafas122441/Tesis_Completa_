/**
 * @file auto_box_calibration.h
 * @brief Consignas de las dos cajas y pulso de inclinacion para descargar.
 * XYZ se refiere a la plataforma DELTA, no al filo de la pala. La configuracion
 * no certifica ausencia de colisiones ni habilita movimiento por si sola.
 */
#ifndef AUTO_BOX_CALIBRATION_H
#define AUTO_BOX_CALIBRATION_H


namespace AutoBoxCalibration {
constexpr unsigned COUNT = 2u;
constexpr unsigned PIECE_ID[COUNT] = {1u, 2u};
constexpr unsigned CAPTURE_RAW[COUNT][3] = {
  {610u, 318u, 278u},
  {615u, 279u, 380u}
};

// Each box uses its own final XYZ; rounded Serial coordinates are references,
// not measured scoop-tip positions or a collision-clearance certificate.
constexpr float TARGET_MM[COUNT][3] = {
  {-23.68f, 210.10f, -198.20f},  
  { 60.83f, 207.21f, -212.87f}  // pieza_chica, latest paired manual capture
};
constexpr float MAX_DEPOSIT_Z_MM = -198.20f;

constexpr unsigned DUMP_X_US = 2070u;
static_assert(DUMP_X_US >= 500u && DUMP_X_US <= 2500u,
              "Automatic dump pulse must remain inside the RC range");
static_assert(TARGET_MM[0][2] <= MAX_DEPOSIT_Z_MM &&
              TARGET_MM[1][2] <= MAX_DEPOSIT_Z_MM,
              "Highest deposit reference must cover both deposit poses");

// IK of TARGET_MM rounded to integer RAW, independently matching these logs.
// Runtime must plan TARGET_MM and revalidate IK/FK/RAW/Jacobian, not jump
// directly to these angles or claim exact equal height after quantization.
constexpr unsigned ROUNDED_RAW[COUNT][3] = {
  {610u, 318u, 278u},
  {615u, 279u, 380u}
};
}
#endif
