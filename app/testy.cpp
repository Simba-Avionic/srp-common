#include <iostream>
#include <filesystem>
#include <string>
#include <cstdlib>

namespace fs = std::filesystem;

int main()
{
    const char *workspace_dir = std::getenv("BUILD_WORKSPACE_DIRECTORY");

    if (workspace_dir == nullptr)
    {
        return 1;
    }

    fs::path base_path = workspace_dir;
    fs::path data_path = base_path / "data";

    fs::path executable_path = base_path / "bazel-bin" / "app" / "csv_apogee_test";

    if (!fs::exists(data_path) || !fs::is_directory(data_path))
    {
        std::cerr << "Błąd: Folder " << data_path << " nie istnieje\n";
        return 1;
    }

    if (!fs::exists(executable_path))
    {
        std::cerr << "Błąd: Plik wykonywalny " << executable_path << " nie istnieje.\n";
        return 1;
    }
    for (const auto &entry : fs::directory_iterator(data_path))
    {

        if (entry.is_directory())
        {
            fs::path barometr_file = entry.path() / "exported_barometer_data.csv";
            fs::path akcelerometr_file = entry.path() / "exported_accel_data.csv";
            fs::path config_file = entry.path() / "config.txt";

            if (fs::exists(barometr_file) && fs::exists(akcelerometr_file))
            {
                std::string command = executable_path.string() +
                                      " \"" + barometr_file.string() + "\"" +
                                      " \"" + akcelerometr_file.string() + "\""+
                                      " \"" + config_file.string() + "\"";

                std::cout << "----------------------------------------\n";
                std::cout << "Testowanie dla danych: " << entry.path().filename() << "\n";

                int result = std::system(command.c_str());

                if (result != 0)
                {
                    std::cerr << "Ostrzeżenie: Program csv_apogee_test zwrócił kod błędu (" << result << ") dla " << entry.path().filename() << "\n";
                }
            }
            else
            {
                std::cout << "----------------------------------------\n";
                std::cerr << "Pominięto folder: " << entry.path().filename();
            }
        }
    }

    std::cout << "----------------------------------------\n";
    std::cout << "Zakończono wszystkie testy.\n";

    return 0;
}