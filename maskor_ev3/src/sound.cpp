#include <maskor_ev3/sound.h>

#include <sstream>

namespace maskor_ev3 {

void sound::beep(const std::string &args, bool bSynchronous)
{
  std::ostringstream cmd;
  cmd << "/usr/bin/beep " << args;
  if (!bSynchronous) cmd << " &";
  std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void sound::tone(
    const std::vector< std::vector<float> > &sequence,
    bool bSynchronous
    )
{
  std::ostringstream args;
  bool first = true;

  for(auto v : sequence) {
    if (first) {
      first = false;
    } else {
      args << " -n";
    }

    if (v.size() > 0) {
      args << " -f " << v[0];
    } else {
      continue;
    }

    if (v.size() > 1) {
      args << " -l " << v[1];
    } else {
      continue;
    }

    if (v.size() > 2) {
      args << " -D " << v[2];
    } else {
      continue;
    }
  }

  beep(args.str(), bSynchronous);
}

//-----------------------------------------------------------------------------

void sound::tone(float frequency, float ms, bool bSynchronous) {
  tone({{frequency, ms, 0.0f}}, bSynchronous);
}

//-----------------------------------------------------------------------------

void sound::play(const std::string &soundfile, bool bSynchronous)
{
  std::ostringstream cmd;
  cmd << "/usr/bin/aplay -q " << soundfile;

  if (!bSynchronous) cmd << " &";

  std::system(cmd.str().c_str());
}

//-----------------------------------------------------------------------------

void sound::speak(const std::string &text, bool bSynchronous)
{
  std::ostringstream cmd;

  cmd << "/usr/bin/espeak -a 200 --stdout \"" << text << "\""
      << " | /usr/bin/aplay -q";

  if (!bSynchronous) cmd << " &";

  std::system(cmd.str().c_str());
}

}//end namespace
