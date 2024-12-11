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

	Quaternion rotation = MakeRotateAxisAngleQuaternion(
		Normalize(Vector3{ 1.0f,0.4f,-0.2f }), 0.45f);
	Vector3 pointY = { 2.1f,-0.9f,1.3f };
	Matrix4x4 rotateMatrix = MakeRotateMatrix(rotation);
	Vector3 rotateByquaternion = RotateVector(pointY, rotation);
	Vector3 rotateByMatrix = TransformCoordinates(pointY, rotateMatrix);

	const uint32_t kRowHeight = 20;

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

		QuaternionScreenPrint(10, kRowHeight * 0, rotation, "rotation");
		MatrixScreenPrintf(10, kRowHeight * 1, rotateMatrix, "rotateMatrix");
		VectorScreenPrintf(10, kRowHeight * 6, rotateByquaternion, "rotateByquaternion");
		VectorScreenPrintf(10, kRowHeight * 7, rotateByMatrix, "rotateByMatrix");


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
