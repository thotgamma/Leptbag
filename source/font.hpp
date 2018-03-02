#ifndef FONT_HPP
#define FONT_HPP

#include <iostream>
#include <vector>
#include <unordered_map>
#include <memory>

#include <ft2build.h>
#include FT_FREETYPE_H

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/glm.hpp>

#include "shader.hpp"


class textbox_interface {
	public:
	virtual void updateText(char16_t * text, int length) = 0;
	virtual void updateColor(int newR, int newG, int newB) = 0;
	virtual void updateSize(int newSize) = 0;
	virtual void updatePos(int newX, int newY) = 0;
	virtual void destroy() = 0;
};

extern "C" textbox_interface* createTextbox_interface(char16_t* text, int length, int x, int y, int size, int r, int g, int b);


class textbox : public textbox_interface {
	const int windowWidth = 1000;
	const int windowHeight = 800;

	// # std::unordered_map<int, textbox*> textbox::ownerList
	// どのテキストボックスが描画リスト何番目の文字を所有しているのかを管理する
	// 描画リストの並び替え(削除時に発生する)の際、持ち主テキストボックスに通知するのに使用する
	static std::unordered_map<int, textbox*> ownerList;

	// テキストのプロパティ
	std::u16string text;
	int x;
	int y;
	int size;
	int r;
	int g;
	int b;
	int length;

	// このクラス(textbox)が管理する文字が文字描画リストの何番目に今あるのかを管理する
	std::unique_ptr<int[]> characterIDArray;

	public:
	textbox(std::u16string text, int x, int y, int size, int r, int g, int b);
	virtual void updateText(char16_t *text, int length);
	virtual void updateColor(int newR, int newG, int newB);
	virtual void updateSize(int newSize);
	virtual void updatePos(int newX, int newY);
	virtual void destroy();

	//void render() : これは実際に画面に描画する訳ではなく、
	//                描画するために頂点情報を計算して記録する関数である。(ネーミングが下手)
	void render();
	void updateID(int before, int after);
};


namespace font {

	// 一頂点を表現
	struct xyuvrgb {
		float x;
		float y;
		float u;
		float v;
		float r;
		float g;
		float b;
	};

	// 一文字を画面に表示するための"長方形"を表現
	struct character {
		xyuvrgb v0;
		xyuvrgb v1;
		xyuvrgb v2;
		xyuvrgb v3;
		xyuvrgb v4;
		xyuvrgb v5;
	};

	// Free Typeからの文字情報を格納する。
	// テクスチャアトラスのどの位置にこの文字が存在するのか、
	// 文字の幅・高さ・ベースラインからの位置等が記録されている。
	// この情報を正しく使うことで、きれいに文字を並べることができる。
	struct charInfo {
		//10 11
		//00 01
		float u00;
		float v00;
		float u01;
		float v01;
		float u10;
		float v10;
		float u11;
		float v11;
		unsigned int width;
		unsigned int height;
		unsigned int bearingX;
		unsigned int bearingY;
		unsigned int advanceWidth;
	};

	// テクスチャアトラスの高さ。この大きさに合うようフォントをFreeTypeでレンダするので、
	// これを大きくすることできれいな文字を描画できる(ハズ)。
	const GLuint textureHeight = 128;

	extern GLuint textAtlas;

	extern FT_Library ft;
	extern FT_Face face;

	extern GLuint program_TEXT;
	extern GLuint characterVAO, characterVBO;

	// テクスチャアトラスに順番に文字を配置するために使用する。
	extern GLuint textAtlasItr;

	// テクスチャアトラス上のUV座標を計算するのに用いる。
	// 管理する文字が多くなると、この値は自動で大きくなる。(初期値はcppに記載)
	extern GLuint textAtlasWidth;

	// 描画する文字のリスト。ここにcharacterを入れることで、画面に実際に描画される。
	extern std::vector<character> characterVector;

	// 文字情報を高速に取得するために使う。
	// Unicodeから文字の幅などの情報を引ける。
	extern std::unordered_map<char16_t, charInfo> charMap;



	//字形情報を返す。フォントレンダからの字形登録までやってくれる。
	charInfo getCharInfo(char16_t request);

	void reloadVBO();

	// アトラステクスチャの大きさが不足した際、二倍の大きさにする
	void expandTexture();

	// characterを投げるとリストに追加してくれる。
	// その時、今登録したcharacterが、リストの何番目に入ったのか通知してくれる。
	// この関数を拡張すれば、複雑な並び替えなどしなくていいのでは🤔 #TODO
	int addCharacterToDrawList(character request);

	// id番目のcharacterをリストから消去する。
	// その時、削除した場所に移動されたcharacterの元idを返す。
	// この関数を呼んだときは、ownerListを使って移動されたcharacterの
	// 持ち主textBoxに通知しなければならない。
	int removeCharacterFromDrawList(int id);

	// 呼んで。まず。
	void setup();

	// 実際に画面に描画する。
	void draw();

}


#endif
