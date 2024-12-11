#pragma once
#include "MathFunc.h"
static const int kRowHeight = 20;
static const int kColumnWidth = 60;
void MatrixScreenPrintf(int x, int y, const Matrix4x4& matrix, const char* name) {
	Novice::ScreenPrintf(x, y, "%s", name);
	for (int row = 0; row < 4; ++row) {
		for (int column = 0; column < 4; ++column) {
			Novice::ScreenPrintf(x + column * kColumnWidth, y + row * kRowHeight + 25, "%6.03f", matrix.m[row][column]);
		}
	}
}

void VectorScreenPrintf(int x, int y, const Vector3& vector, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", vector.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", vector.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", vector.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%s", label);
}

void QuaternionScreenPrint(int x, int y, const Quaternion& q, const char* label) {
	Novice::ScreenPrintf(x, y, "%.02f", q.x);
	Novice::ScreenPrintf(x + kColumnWidth, y, "%.02f", q.y);
	Novice::ScreenPrintf(x + kColumnWidth * 2, y, "%.02f", q.z);
	Novice::ScreenPrintf(x + kColumnWidth * 3, y, "%.02f", q.w);
	Novice::ScreenPrintf(x + kColumnWidth * 4, y, "%s", label);
}

