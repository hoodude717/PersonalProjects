#include "config.h"
#include "triangle_mesh.h"


unsigned int make_module(const std::string& filepath, unsigned int module_type);

unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath);

const std::string vertex_filepath = "../graphics/src/shaders/vertex.txt";
const std::string fragment_filepath = "../graphics/src/shaders/fragment.txt";

int main() {
    std::ifstream file;
    std::string line;

    file.open(vertex_filepath);
    while (std::getline(file, line)) {
        std::cout << line << std::endl;
    }

    if (!glfwInit()) {
        std::cout << "GLFW Couldnt Initialize" << std::endl;
        return -1;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);  // 4.1 is well supported on macOS
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);

    GLFWwindow* window;

    window = glfwCreateWindow(640,480,"Smart Wand Display", NULL, NULL);
    if (!window) {
        std::cout << "Failed to create GLFW window" << std::endl;
        glfwTerminate();
        return -1;
    }
    glfwMakeContextCurrent(window);
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        glfwTerminate();
        return -1;
    }

    glClearColor(0.25f, 0.5f, 0.75f, .50f);
    unsigned int shaderProgram = make_shader(vertex_filepath, fragment_filepath);

    TriangleMesh* triangle = new TriangleMesh();

    while(!glfwWindowShouldClose(window)) {
        glfwPollEvents();
        glClear(GL_COLOR_BUFFER_BIT);
        glUseProgram(shaderProgram);
        triangle->draw();
        glfwSwapBuffers(window);

    }
    glDeleteProgram(shaderProgram);
    glfwTerminate();
    return 0;
}

unsigned int make_shader(const std::string& vertex_filepath, const std::string& fragment_filepath) {
    std::vector<unsigned int> shaderModules;
    shaderModules.push_back(make_module(vertex_filepath, GL_VERTEX_SHADER));
    shaderModules.push_back(make_module(fragment_filepath, GL_FRAGMENT_SHADER));

    unsigned int shaderProgram = glCreateProgram();
    for (unsigned int module : shaderModules) {
        glAttachShader(shaderProgram, module);
    }
    glLinkProgram(shaderProgram);

    int success; 
    glGetProgramiv(shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetProgramInfoLog(shaderProgram, 1024, NULL, errorLog);
        std::cout << "Error linking shader program: " << std::endl << errorLog ;
    }

    for (unsigned int module : shaderModules) {
        glDeleteShader(module);
    }
    return shaderProgram;
}

unsigned int make_module(const std::string& filepath, unsigned int module_type) {
    std::ifstream file;
    std::stringstream bufferLines;
    std::string line;

    file.open(filepath);
    while (std::getline(file, line)) {
        bufferLines << line << "\n";
    }
    std::string shaderSource = bufferLines.str();
    const char* shaderCode = shaderSource.c_str();
    bufferLines.clear();
    file.close();

    unsigned int shaderModule = glCreateShader(module_type);
    if (shaderModule == 0) {
        std::cout << "Failed to create shader object for type: " << module_type << std::endl;
        return 0;
    }
    glShaderSource(shaderModule, 1, &shaderCode, NULL);
    glCompileShader(shaderModule);

    int success; 
    glGetShaderiv(shaderModule, GL_COMPILE_STATUS, &success);
    if (!success) {
        char errorLog[1024];
        glGetShaderInfoLog(shaderModule, 1024, NULL, errorLog);
        std::cout << "Error compiling shader module: " << errorLog << std::endl;
    }
    return shaderModule;
}
