#include "Shader.h"

#include <QString>
#include <QFile>
#include <QDebug>

Shader::Shader()
	: program_(0) {}

Shader::~Shader() {
	auto fn = functions();

	for (const auto& shaderObject : shaderObjects_) {
		fn->glDetachShader(program_, shaderObject);
		fn->glDeleteShader(shaderObject);
	}
	shaderObjects_.clear();

	if (program_ != 0) {
		fn->glDeleteProgram(program_);
		program_ = 0;
	}
}

bool Shader::init() {
	auto fn = functions();

	program_ = fn->glCreateProgram();

	const auto error = fn->glGetError();
	if (error != GL_NO_ERROR) {
		qDebug().noquote() << QString("Error calling Shader::init(): %1").arg(error);
	}

	return error == GL_NO_ERROR;
}

bool Shader::addSource(GLenum shaderType, const QString& shaderSource) {
	auto fn = functions();

	const QString errorFormat =
		QString("Error calling Shader::addSource(shaderType=%1, shaderSource=\"...\"): %2")
			.arg(shaderType);

	const auto	  source	 = shaderSource.toLatin1();
	const GLchar* sources[1] = {source.data()};
	GLint		  lengths[1] = {static_cast<GLint>(source.size())};

	GLuint shaderObject = fn->glCreateShader(shaderType);
	if (shaderObject == 0) {
		qDebug().noquote() << errorFormat.arg("cannot create shader object");
		return false;
	} else {
		shaderObjects_.push_back(shaderObject);
	}

	fn->glShaderSource(shaderObject, 1, sources, lengths);
	fn->glCompileShader(shaderObject);

	GLint success;
	fn->glGetShaderiv(shaderObject, GL_COMPILE_STATUS, &success);
	if (!success) {
		GLchar infoLog[1024];
		fn->glGetShaderInfoLog(shaderObject, sizeof(infoLog), nullptr, infoLog);
		qDebug().noquote() << errorFormat.arg(QString("error compiling shader: %1").arg(infoLog));
		return false;
	}

	fn->glAttachShader(program_, shaderObject);
	return true;
}

bool Shader::addSourceFile(GLenum shaderType, const QString& shaderPath) {
	const QString errorFormat =
		QString("Error calling Shader::addSourceFile(shaderType=%1, shaderPath=%2): %3")
			.arg(shaderType)
			.arg(QString("\"%1\"").arg(shaderPath));

	QFile shaderFile(shaderPath);
	if (!shaderFile.open(QFile::ReadOnly)) {
		qDebug().noquote() << errorFormat.arg("cannot open target shader file");
		return false;
	}

	const auto shaderSource = shaderFile.readAll();
	shaderFile.close();

	return addSource(shaderType, shaderSource);
}

bool Shader::finalize() {
	auto		  fn		  = functions();
	const QString errorFormat = QString("Error calling Shader::finalize(): %1");

	fn->glLinkProgram(program_);

	GLint success;
	fn->glGetProgramiv(program_, GL_LINK_STATUS, &success);
	if (!success) {
		GLchar infoLog[1024];
		fn->glGetProgramInfoLog(program_, sizeof(infoLog), nullptr, infoLog);
		qDebug().noquote() << errorFormat.arg(
			QString("error linking shader program: %1").arg(infoLog));
		return false;
	}

	for (const auto& shaderObject : shaderObjects_) {
		fn->glDetachShader(program_, shaderObject);
		fn->glDeleteShader(shaderObject);
	}
	shaderObjects_.clear();

	fn->glValidateProgram(program_);
	fn->glGetProgramiv(program_, GL_VALIDATE_STATUS, &success);
	if (!success) {
		GLchar infoLog[1024];
		fn->glGetProgramInfoLog(program_, sizeof(infoLog), nullptr, infoLog);
		qDebug().noquote() << errorFormat.arg(
			QString("error validating shader program: %1").arg(infoLog));
		return false;
	}

	return true;
}

bool Shader::bind() {
	auto fn = functions();

	fn->glUseProgram(program_);

	const auto error = fn->glGetError();
	if (error != GL_NO_ERROR) {
		qDebug().noquote() << QString("Error calling Shader::use(): %1").arg(error);
		return false;
	}

	return true;
}

void Shader::release() {
	auto fn = functions();

	fn->glUseProgram(0);
}

GLint Shader::getParameter(GLenum paramName) {
	auto fn = functions();

	GLint value = 0;
	fn->glGetProgramiv(program_, paramName, &value);
	return value;
}

bool Shader::setParameter(GLenum paramName, GLint value) {
	auto fn = extraFunctions();

	fn->glProgramParameteri(program_, paramName, value);
	return fn->glGetError() == GL_NO_ERROR;
}

GLint Shader::uniformLocation(const char* uniformName) {
	auto fn = functions();

	return fn->glGetUniformLocation(program_, uniformName);
}
