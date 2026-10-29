#include <iostream>
#include "plaits/dsp/voice.h"

int main() {
    std::cout << "Size of plaits::Voice: " << sizeof(plaits::Voice) << " bytes" << std::endl;
    std::cout << "Size of plaits::SpeechEngine: " << sizeof(plaits::SpeechEngine) << " bytes" << std::endl;
    std::cout << "Size of plaits::SixOpEngine: " << sizeof(plaits::SixOpEngine) << " bytes" << std::endl;
    std::cout << "Size of plaits::WaveTerrainEngine: " << sizeof(plaits::WaveTerrainEngine) << " bytes" << std::endl;
    return 0;
}
