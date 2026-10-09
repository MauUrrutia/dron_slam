#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <libserial/SerialPort.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <string>
#include <vector>

using namespace std::chrono_literals;

constexpr float PI = 3.14159265358979323846f;

constexpr float degToRad(float deg)
{
  return deg * PI / 180.0f;
}

class TeensySerialReceiver : public rclcpp::Node
{
public:
  TeensySerialReceiver();
  ~TeensySerialReceiver();

private:
  void serialTimerCallback();
  void processFrame(const uint8_t * buffer);
  void publishMergedPointCloud();

  void writeNaN(
    sensor_msgs::PointCloud2Iterator<float> & iter_x,
    sensor_msgs::PointCloud2Iterator<float> & iter_y,
    sensor_msgs::PointCloud2Iterator<float> & iter_z);

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;

  // Timer para leer el puerto serial.
  rclcpp::TimerBase::SharedPtr serial_timer_;

  // Timer independiente para publicar la nube.
  rclcpp::TimerBase::SharedPtr cloud_timer_;

  std::string port_;
  LibSerial::SerialPort teensy_;

  std::vector<uint8_t> rx_buffer_;

  static constexpr std::size_t FRAME_SIZE = 132;

  // Número de canales configurados.
  static constexpr int NUM_CHANNELS = 5;

  // VL53L7CX: 8 x 8 zonas.
  static constexpr int PTS_PER_SENSOR = 64;

  // Distancias más recientes recibidas por cada canal.
  std::array<std::array<uint16_t, PTS_PER_SENSOR>, NUM_CHANNELS>
    latest_distances_{};

  // Indica si ya se recibió al menos un frame de cada canal.
  std::array<bool, NUM_CHANNELS> has_data_{
    {false, false, false, false, false}
  };

  // Indica si ya se recibió algún dato.
  bool received_any_data_{false};

  /*
   * Orientación de cada sensor respecto a base_footprint.

   * Cada sensor tiene aproximadamente 60 grados de FOV.
   *
   * Canal 0: -30 a +30 grados
   * Canal 1: -90 a -30 grados
   * Canal 2: +30 a +90 grados
   * Canal 3: +90 a +150 grados
   * Canal 4: -150 a -90 grados
   *
   * En total se cubren aproximadamente 300 grados.
   */
  std::array<float, NUM_CHANNELS> sensor_yaw_deg_{
    0.0f,
    -60.0f,
    60.0f,
    120.0f,
    -120.0f
  };

  /*
   * Distancia de cada sensor al centro de base_footprint,
   * expresada en metros.

   * Por ejemplo:
   * 0.05 m = 5 cm.
   *
   * Si los sensores están prácticamente en el centro,
   * puedes dejar estos valores en 0.0f.
   */
  std::array<float, NUM_CHANNELS> sensor_radius_m_{
    0.00f,
    0.00f,
    0.00f,
    0.00f,
    0.00f
  };
};

TeensySerialReceiver::TeensySerialReceiver()
: Node("teensy_serial_receiver")
{
  declare_parameter<std::string>("port", "/dev/ttyACM0");
  port_ = get_parameter("port").as_string();

  try {
    teensy_.Open(port_);
    teensy_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);

    RCLCPP_INFO(
      get_logger(),
      "Puerto serial abierto correctamente: %s",
      port_.c_str());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_logger(),
      "No se pudo abrir el puerto serial %s: %s",
      port_.c_str(),
      e.what());

    throw;
  }

  pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
    "/sensor_data",
    rclcpp::SensorDataQoS());

  /*
   * Este timer solo lee y procesa los frames seriales.
   * 10 ms equivale aproximadamente a 100 Hz.
   */
  serial_timer_ = create_wall_timer(
    10ms,
    std::bind(
      &TeensySerialReceiver::serialTimerCallback,
      this));

  /*
   * Este timer publica la nube fusionada.
   * 50 ms equivale aproximadamente a 20 Hz.
   *
   * Publicar a 20 Hz es suficiente para visualización
   * y evita saturar RViz/DDS.
   */
  cloud_timer_ = create_wall_timer(
    50ms,
    std::bind(
      &TeensySerialReceiver::publishMergedPointCloud,
      this));

  rx_buffer_.reserve(FRAME_SIZE * 4);
}

TeensySerialReceiver::~TeensySerialReceiver()
{
  try {
    if (teensy_.IsOpen()) {
      teensy_.Close();
    }
  } catch (const std::exception & e) {
    RCLCPP_WARN(
      get_logger(),
      "Error al cerrar el puerto serial: %s",
      e.what());
  }
}

void TeensySerialReceiver::serialTimerCallback()
{
  /*
   * Leer todos los bytes disponibles del puerto serial.
   */
  while (teensy_.IsDataAvailable()) {
    char c;
    teensy_.ReadByte(c);

    rx_buffer_.push_back(
      static_cast<uint8_t>(c));
  }

  /*
   * Buscar y procesar todos los frames completos
   * actualmente disponibles en el buffer.
   */
  while (true) {
    /*
     * Buscar el byte de inicio 0xAA.
     */
    auto start_it = std::find(
      rx_buffer_.begin(),
      rx_buffer_.end(),
      static_cast<uint8_t>(0xAA));

    /*
     * Si no se encontró el inicio, descartar el buffer.
     */
    if (start_it == rx_buffer_.end()) {
      rx_buffer_.clear();
      return;
    }

    /*
     * Eliminar cualquier basura anterior al byte 0xAA.
     */
    if (start_it != rx_buffer_.begin()) {
      rx_buffer_.erase(
        rx_buffer_.begin(),
        start_it);
    }

    /*
     * Todavía no hay un frame completo.
     */
    if (rx_buffer_.size() < FRAME_SIZE) {
      return;
    }

    /*
     * Verificar el byte final esperado.
     *
     * El código original utiliza buffer[130] como byte final.
     */
    if (rx_buffer_[130] != static_cast<uint8_t>(0xBB)) {
      /*
       * El supuesto frame no es válido.
       * Descartar solamente el primer byte y buscar
       * el siguiente posible inicio.
       */
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    /*
     * Copiar el frame completo.
     */
    std::array<uint8_t, FRAME_SIZE> frame{};

    std::copy(
      rx_buffer_.begin(),
      rx_buffer_.begin() + FRAME_SIZE,
      frame.begin());

    processFrame(frame.data());

    /*
     * Eliminar el frame ya procesado.
     */
    rx_buffer_.erase(
      rx_buffer_.begin(),
      rx_buffer_.begin() + FRAME_SIZE);
  }
}

void TeensySerialReceiver::processFrame(const uint8_t * buffer)
{
  /*
   * Verificación básica del frame.
   */
  if (buffer[0] != static_cast<uint8_t>(0xAA) ||
      buffer[130] != static_cast<uint8_t>(0xBB)) {
    return;
  }

  /*
   * El canal se encuentra en buffer[1].
   */
  const uint8_t channel = buffer[1];

  /*
   * Ignorar canales fuera del rango 0..4.
   */
  if (channel >= NUM_CHANNELS) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Se recibió un canal inválido: %u",
      channel);

    return;
  }

  /*
   * Extraer las 64 distancias.
   *
   * Cada distancia ocupa dos bytes:
   * - byte bajo
   * - byte alto
   */
  std::array<uint16_t, PTS_PER_SENSOR> distances_mm{};

  for (std::size_t i = 0; i < PTS_PER_SENSOR; ++i) {
    const std::size_t byte_index = 2 + i * 2;

    distances_mm[i] =
      static_cast<uint16_t>(buffer[byte_index]) |
      (static_cast<uint16_t>(buffer[byte_index + 1]) << 8);
  }

  /*
   * Guardar las mediciones más recientes del canal.
   */
  latest_distances_[channel] = distances_mm;
  has_data_[channel] = true;
  received_any_data_ = true;

  /*
   * No publicar aquí.
   *
   * La publicación se realiza desde cloud_timer_
   * a una frecuencia fija de 20 Hz.
   */
}

void TeensySerialReceiver::writeNaN(
  sensor_msgs::PointCloud2Iterator<float> & iter_x,
  sensor_msgs::PointCloud2Iterator<float> & iter_y,
  sensor_msgs::PointCloud2Iterator<float> & iter_z)
{
  const float nan_value =
    std::numeric_limits<float>::quiet_NaN();

  *iter_x = nan_value;
  *iter_y = nan_value;
  *iter_z = nan_value;

  ++iter_x;
  ++iter_y;
  ++iter_z;
}

void TeensySerialReceiver::publishMergedPointCloud()
{
  /*
   * No publicar nubes vacías al iniciar el nodo.
   */
  if (!received_any_data_) {
    return;
  }

  constexpr float FOV_DEG = 60.0f;
  constexpr float STEP_RAD =
    degToRad(FOV_DEG) / 8.0f;

  constexpr float MM_TO_M = 0.001f;
  constexpr float MAX_RANGE_M = 4.0f;

  /*
   * Se mantienen 64 puntos por cada uno de los 5 canales.
   *
   * Si solamente tienes 3 sensores físicos,
   * los canales que nunca reciban datos serán NaN.
   */
  sensor_msgs::msg::PointCloud2 cloud;

  cloud.header.frame_id = "base_footprint";
  cloud.header.stamp = now();

  /*
   * Se utiliza una nube no organizada:
   * height = 1
   * width = número total de puntos
   */
  cloud.height = 1;
  cloud.width = NUM_CHANNELS * PTS_PER_SENSOR;

  cloud.is_dense = false;
  cloud.is_bigendian = false;

  /*
   * Crear campos x, y, z.
   */
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2FieldsByString(1, "xyz");
  modifier.resize(cloud.width);

  sensor_msgs::PointCloud2Iterator<float> iter_x(
    cloud,
    "x");

  sensor_msgs::PointCloud2Iterator<float> iter_y(
    cloud,
    "y");

  sensor_msgs::PointCloud2Iterator<float> iter_z(
    cloud,
    "z");

  /*
   * Procesar cada canal.
   */
  for (int ch = 0; ch < NUM_CHANNELS; ++ch) {
    /*
     * Si todavía no se ha recibido información del canal,
     * llenar sus 64 puntos con NaN.
     */
    if (!has_data_[ch]) {
      for (int i = 0; i < PTS_PER_SENSOR; ++i) {
        writeNaN(iter_x, iter_y, iter_z);
      }

      continue;
    }

    /*
     * Orientación horizontal del sensor.
     */
    const float yaw =
      degToRad(sensor_yaw_deg_[ch]);

    const float cos_yaw =
      std::cos(yaw);

    const float sin_yaw =
      std::sin(yaw);

    /*
     * Posición del sensor sobre el círculo.
     */
    const float radius =
      sensor_radius_m_[ch];

    const float sensor_offset_x =
      radius * cos_yaw;

    const float sensor_offset_y =
      radius * sin_yaw;

    /*
     * Procesar las 64 zonas del VL53L7CX.
     */
    for (int row = 0; row < 8; ++row) {
      for (int col = 0; col < 8; ++col) {
        const int idx = row * 8 + col;

        const float depth_m =
          latest_distances_[ch][idx] * MM_TO_M;

        /*
         * Ángulo horizontal y vertical de cada zona.
         */
        const float angle_h =
          (static_cast<float>(col) - 3.5f) *
          STEP_RAD;

        const float angle_v =
          (static_cast<float>(row) - 3.5f) *
          STEP_RAD;

        /*
         * Estos signos dependen de cómo esté orientado
         * físicamente el sensor.
         *
         * Si la nube aparece invertida horizontalmente,
         * cambia corrected_h = angle_h.
         *
         * Si aparece invertida verticalmente,
         * cambia corrected_v = angle_v.
         */
        const float corrected_h = -angle_h;
        const float corrected_v = -angle_v;

        /*
         * Filtrar mediciones inválidas.
         */
        if (depth_m <= 0.0f ||
            depth_m > MAX_RANGE_M) {
          writeNaN(iter_x, iter_y, iter_z);
          continue;
        }

        /*
         * Coordenadas locales del sensor:
         *
         * x: adelante
         * y: lateral
         * z: arriba
         */
        const float x_local = depth_m;

        const float y_local =
          depth_m * std::tan(corrected_h);

        const float z_local =
          depth_m * std::tan(corrected_v);

        /*
         * Rotación horizontal alrededor de Z.
         */
        const float x_rotated =
          cos_yaw * x_local -
          sin_yaw * y_local;

        const float y_rotated =
          sin_yaw * x_local +
          cos_yaw * y_local;

        /*
         * Transformación final a base_footprint.
         */
        const float x_global =
          x_rotated + sensor_offset_x;

        const float y_global =
          y_rotated + sensor_offset_y;

        const float z_global =
          z_local;

        *iter_x = x_global;
        *iter_y = y_global;
        *iter_z = z_global;

        ++iter_x;
        ++iter_y;
        ++iter_z;
      }
    }
  }

  /*
   * Publicar una sola nube a la frecuencia
   * definida por cloud_timer_.
   */
  pub_->publish(cloud);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<TeensySerialReceiver>();

  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}