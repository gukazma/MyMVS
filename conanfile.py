from conans import ConanFile, tools
import platform


class ModuleConan(ConanFile):
    name = "QtConanExample"
    description = "An example for Qt with Conan"
    settings = "os", "compiler", "build_type", "arch"
    generators = "qt", "cmake", "cmake_find_package_multi", "cmake_paths"
    default_options = {
        "qt:shared": True,
        "qt:qttools": True
    }

    def configure(self):
        del self.settings.compiler.cppstd

    def requirements(self):
        self.requires("gtest/1.14.0")
        self.requires("qt/5.15.3")
        self.requires("openmvg/2.0")
        self.requires("zlib/1.3", override=True)
        self.requires("libjpeg/9e", override=True)
        self.requires("libpng/1.6.40", override=True)
        self.requires("zstd/1.5.5", override=True)
        
    def imports(self):
        self.copy("*.dll", "./bin", "bin")
