// Exercise the installed MOLA/MRPT YAML parser, without starting ROS or SLAM.
#include <mola_yaml/yaml_helpers.h>
#include <iostream>
#include <iterator>
#include <string>

int main()
{
    try
    {
        const std::string text(std::istreambuf_iterator<char>{std::cin}, {});
        const auto raw = mrpt::containers::yaml::FromText(text);
        const auto parsed = mola::parse_yaml(raw);
        std::cout << mola::yaml_to_string(parsed);
        return 0;
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        return 1;
    }
}
