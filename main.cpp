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


	// ウィンドウの×ボタンが押されるまでループ
	while (Novice::ProcessMessage() == 0) {
		// フレームの開始
		Novice::BeginFrame();

		// キー入力を受け取る
		memcpy(preKeys, keys, 256);
		Novice::GetHitKeyStateAll(keys);


		Quaternion q1 =  {2.0f, 3.0f, 4.0f, 1.0f};
		Quaternion q2 = { 1.0f, 3.0f, 5.0f, 2.0f };
		Quaternion identity = IdentityQuaternion();
		Quaternion conj =  Conjugate(q1);
		Quaternion inv = Inverse(q1);
		Quaternion normal = Normalize(q1);
		Quaternion mul1  = Multiply(q1, q2);
		Quaternion mul2 = Multiply(q2, q1);
		float norm = Norm(q1);

		///
		/// ↓更新処理ここから
		///

		


		///
		/// ↑更新処理ここまで
		///

		///
		/// ↓描画処理ここから
		///

		QuaternionScreenPrint(10, 0, identity, "Identity");
		QuaternionScreenPrint(10, 20, conj, "Conjugate");
		QuaternionScreenPrint(10, 40, inv, "Inverse");
		QuaternionScreenPrint(10, 60, normal, "Normalize");
		QuaternionScreenPrint(10, 80, mul1, "Multiply(q1,q2)");
		QuaternionScreenPrint(10, 100, mul2, "Multiply(q2,q1)");
		Novice::ScreenPrintf(10, 120, "%0.2f : Norm", norm);
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
