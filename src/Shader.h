#pragma once

#include <QOpenGLFunctions>
#include <QOpenGLExtraFunctions>
#include <type_traits>
#include <list>

class Shader {
public:
	Shader();
	virtual ~Shader();

	bool init();
	bool addSource(GLenum shaderType, const QString &shaderSource);
	bool addSourceFile(GLenum shaderType, const QString& shaderPath);
	bool finalize();
	bool bind();
	void release();

	GLint getParameter(GLenum paramName);
	bool  setParameter(GLenum paramName, GLint value);

	GLint uniformLocation(const char* uniformName);

	template <int Rows, int Cols, bool transpose = false, typename LocationType = GLuint>
		requires((std::is_integral_v<LocationType> ||
				  std::is_same_v<
					  std::remove_cv_t<std::remove_pointer_t<std::decay_t<LocationType>>>, char>) &&
				 (Rows >= 2 && Rows <= 4) && (Cols >= 2 && Cols <= 4))
	inline void setUniform(LocationType loc, float* data) {
		GLuint location = 0;
		if constexpr (std::is_integral_v<LocationType>) {
			location = location;
		} else {
			location = uniformLocation(loc);
		}

		auto fn = extraFunctions();

		if constexpr (Rows == 2) {
			if constexpr (Cols == 2) {
				fn->glProgramUniformMatrix2fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 3) {
				fn->glProgramUniformMatrix2x3fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 4) {
				fn->glProgramUniformMatrix2x4fv(program_, location, 1, transpose, data);
			}
		} else if constexpr (Rows == 3) {
			if constexpr (Cols == 2) {
				fn->glProgramUniformMatrix3x2fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 3) {
				fn->glProgramUniformMatrix3fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 4) {
				fn->glProgramUniformMatrix3x4fv(program_, location, 1, transpose, data);
			}
		} else if constexpr (Rows == 4) {
			if constexpr (Cols == 2) {
				fn->glProgramUniformMatrix4x2fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 3) {
				fn->glProgramUniformMatrix4x3fv(program_, location, 1, transpose, data);
			} else if constexpr (Cols == 4) {
				fn->glProgramUniformMatrix4fv(program_, location, 1, transpose, data);
			}
		}
	}

	template <typename... T, typename LocationType = GLuint>
		requires((std::is_integral_v<LocationType> ||
				  std::is_same_v<
					  std::remove_cv_t<std::remove_pointer_t<std::decay_t<LocationType>>>, char>))
	inline void setUniform(LocationType loc, T... value) {
		using value_type   = std::decay_t<decltype(std::get<0>(std::tuple<T...>()))>;
		constexpr size_t N = sizeof...(T);

		GLuint location = 0;
		if constexpr (std::is_integral_v<LocationType>) {
			location = location;
		} else {
			location = uniformLocation(loc);
		}

		static_assert(std::is_arithmetic_v<value_type>, "invalid value type of glUniform...");

		auto fn = extraFunctions();

		if constexpr (std::is_floating_point_v<value_type>) {
			if constexpr (N == 1) {
				fn->glProgramUniform1f(program_, location, value...);
			} else if constexpr (N == 2) {
				fn->glProgramUniform2f(program_, location, value...);
			} else if constexpr (N == 3) {
				fn->glProgramUniform3f(program_, location, value...);
			} else if constexpr (N == 4) {
				fn->glProgramUniform4f(program_, location, value...);
			}
		} else if constexpr (std::is_signed_v<value_type>) {
			if constexpr (N == 1) {
				fn->glProgramUniform1i(program_, location, value...);
			} else if constexpr (N == 2) {
				fn->glProgramUniform2i(program_, location, value...);
			} else if constexpr (N == 3) {
				fn->glProgramUniform3i(program_, location, value...);
			} else if constexpr (N == 4) {
				fn->glProgramUniform4i(program_, location, value...);
			}
		} else if constexpr (std::is_unsigned_v<value_type>) {
			if constexpr (N == 1) {
				fn->glProgramUniform1ui(program_, location, value...);
			} else if constexpr (N == 2) {
				fn->glProgramUniform2ui(program_, location, value...);
			} else if constexpr (N == 3) {
				fn->glProgramUniform3ui(program_, location, value...);
			} else if constexpr (N == 4) {
				fn->glProgramUniform4ui(program_, location, value...);
			}
		}
	}

	void setUniformBlockBinding(const char *uniformName, GLuint binding) {
		auto fn = extraFunctions();

		GLuint index = fn->glGetUniformBlockIndex(program_, uniformName);
		fn->glUniformBlockBinding(program_, index, binding);
	}

	inline GLuint program() const { return program_; }

protected:
	inline QOpenGLFunctions* functions() { return QOpenGLContext::currentContext()->functions(); }

	inline QOpenGLExtraFunctions* extraFunctions() {
		return QOpenGLContext::currentContext()->extraFunctions();
	}

	GLuint program_;

private:
	std::list<GLuint> shaderObjects_;
};