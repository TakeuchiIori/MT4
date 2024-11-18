#include <Novice.h>
#include "NoviceMath.h"
const char kWindowTitle[] = "MT4_01_01";

// Windowsアプリでのエントリーポイント(main関数)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {

	// ライブラリの初期化
	Novice::Initialize(kWindowTitle, 1280, 720);

	// キー入力結果を受け取る箱
	char keys[256] = {0};
	char preKeys[256] = {0};

	Vector3 from[2] = { Normalize(Vector3{1.0f,0.7f,0.5f}),Normalize(Vector3{-0.6f,0.9f,0.2f}) };
	Vector3 to[2] = { Vector3{-from[0].x,-from[0].y,-from[0].z},Normalize(Vector3{0.4f,0.7f,-0.5f}) };

	Matrix4x4 rotateMatrix[3] = { DirectionToDirection(Normalize(Vector3{1.0f,0.0f,0.0f}),Normalize(Vector3{-1.0f,0.0f,0.0f})),
								 DirectionToDirection(from[0],to[0]),
								 DirectionToDirection(from[1],to[1])
	};

	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);

		///
		/// ↓更新処理ここから
		///

		


		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///
		MatrixScreenPrintf(0, 0, rotateMatrix[0], "Matrix0");

		MatrixScreenPrintf(0, 100, rotateMatrix[1], "Matrix1");

		MatrixScreenPrintf(0, 200, rotateMatrix[2], "Matrix2");
		///
		/// ↑描画処理ここまで
		///

		// フレームの終了
		Novice::EndFrame();

		// ESCキーが押されたらループを抜ける
		if (preKeys[DIK_ESCAPE] == 0 && keys[DIK_ESCAPE] != 0) {
			break;
		}
	}

	// ライブラリの終了
	Novice::Finalize();
	return 0;
}
