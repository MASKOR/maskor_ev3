#ifndef _COMMON_H_
#define _COMMON_H_

#ifndef SYS_ROOT
#define SYS_ROOT "/sys/class"
#endif

#ifndef NO_LINUX_HEADERS
#include <linux/fb.h>
#include <linux/input.h>
#else
#define KEY_CNT 8
#endif

#ifndef FSTREAM_CACHE_SIZE
#define FSTREAM_CACHE_SIZE 16
#endif

#include <set>
#include <string>
#include <mutex>
#include <fstream>
#include <maskor_ev3/lru_cache.h>

static const int bits_per_long = sizeof(long) * 8;

static std::mutex ofstream_cache_lock;
static std::mutex ifstream_cache_lock;

// A global cache of files.
static lru_cache<std::string, std::ifstream> ifstream_cache(FSTREAM_CACHE_SIZE);
static lru_cache<std::string, std::ofstream> ofstream_cache(FSTREAM_CACHE_SIZE);

inline std::ofstream &ofstream_open(const std::string &path)
{
  std::lock_guard<std::mutex> lock(ofstream_cache_lock);
  std::ofstream &file = ofstream_cache[path];
  if (!file.is_open())
  {
    // Don't buffer writes to avoid latency. Also saves a bit of memory.
    file.rdbuf()->pubsetbuf(NULL, 0);
    file.open(path);
  }
  else
  {
    // Clear the error bits in case something happened.
    file.clear();
  }
  return file;
}

inline std::ifstream &ifstream_open(const std::string &path)
{
  std::lock_guard<std::mutex> lock(ifstream_cache_lock);
  std::ifstream &file = ifstream_cache[path];
  if (!file.is_open())
  {
    file.open(path);
  }
  else
  {
    // Clear the flags bits in case something happened (like reaching EOF).
    file.clear();
    file.seekg(0, std::ios::beg);
  }
  return file;
}




namespace maskor_ev3 {

typedef std::string         device_type;
typedef std::string         mode_type;
typedef std::set<mode_type> mode_set;
typedef std::string         address_type;

//-----------------------------------------------------------------------------

const address_type INPUT_AUTO;  //!< Automatic input selection
const address_type OUTPUT_AUTO; //!< Automatic output selection

#ifdef EV3DEV_PLATFORM_BRICKPI
const address_type INPUT_1  { "ttyAMA0:in1" };  //!< Sensor port 1
const address_type INPUT_2  { "ttyAMA0:in2" };  //!< Sensor port 2
const address_type INPUT_3  { "ttyAMA0:in3" };  //!< Sensor port 3
const address_type INPUT_4  { "ttyAMA0:in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "ttyAMA0:outA" }; //!< Motor port A
const address_type OUTPUT_B { "ttyAMA0:outB" }; //!< Motor port B
const address_type OUTPUT_C { "ttyAMA0:outC" }; //!< Motor port C
const address_type OUTPUT_D { "ttyAMA0:outD" }; //!< Motor port D
#else
const address_type INPUT_1  { "in1" };  //!< Sensor port 1
const address_type INPUT_2  { "in2" };  //!< Sensor port 2
const address_type INPUT_3  { "in3" };  //!< Sensor port 3
const address_type INPUT_4  { "in4" };  //!< Sensor port 4

const address_type OUTPUT_A { "outA" }; //!< Motor port A
const address_type OUTPUT_B { "outB" }; //!< Motor port B
const address_type OUTPUT_C { "outC" }; //!< Motor port C
const address_type OUTPUT_D { "outD" }; //!< Motor port D
#endif

} //namespace maskor_ev3

#endif //COMMON_H

