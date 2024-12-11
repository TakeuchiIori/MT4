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

	Quaternion rotation[2] = { 
		MakeRotateAxisAngleQuaternion({ 0.71f,0.71f,0.0f }, 0.3f),
		MakeRotateAxisAngleQuaternion({ 0.71f,0.0f,0.71f },3.141592f), 
	};


	Quaternion interpolate[5] = {
		Slerp(rotation[0],rotation[1],0.0f),
		Slerp(rotation[0],rotation[1],0.3f),
		Slerp(rotation[0],rotation[1],0.5f),
		Slerp(rotation[0],rotation[1],0.7f),
		Slerp(rotation[0],rotation[1],1.0f),
	};
	

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

		
		QuaternionScreenPrint(10, kRowHeight * 0, interpolate[0], "Interpolate[0] : Slerp(q0,q1,0.0f)");
		QuaternionScreenPrint(10, kRowHeight * 1, interpolate[1], "Interpolate[1] : Slerp(q0,q1,0.3f)");
		QuaternionScreenPrint(10, kRowHeight * 2, interpolate[2], "Interpolate[2] : Slerp(q0,q1,0.5f)");
		QuaternionScreenPrint(10, kRowHeight * 3, interpolate[3], "Interpolate[3] : Slerp(q0,q1,0.7f)");
		QuaternionScreenPrint(10, kRowHeight * 4, interpolate[4], "Interpolate[4] : Slerp(q0,q1,1.0f)");


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
