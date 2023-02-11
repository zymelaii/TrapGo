#include "WorldView.h"

#include <QApplication>
#include <QFontDatabase>
#include <filesystem>

namespace fs = std::filesystem;

int main(int argc, char* argv[]) {
	QApplication app(argc, argv);
	for (auto e : fs::directory_iterator("fonts")) {
		QFontDatabase::addApplicationFont(e.path().generic_u8string().c_str());
	}

	WorldView w;

	return app.exec();
}
