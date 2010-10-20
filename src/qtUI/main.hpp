#ifndef MAIN_HPP
#define MAIN_HPP

#if defined( QT_LIBRARY ) && defined( OOMOVE3D_CORE )
#include "qtUI/qtLibrary.hpp"
#endif

#if defined( QT_GL ) && defined( OOMOVE3D_CORE )
#include "qtUI/qtOpenGL/qtGLWindow.hpp"
#endif

/**
 * @ingroup qtWindow
 * @brief Main double thread class (X-Forms Thread)
 */
class Fl_thread: public QThread
{

Q_OBJECT

public:
        int _argc;
        char** _argv;

        Fl_thread(QObject* parent = 0);
        Fl_thread(int argc, char** argv, QObject* parent = 0);

protected:
        void run();

};


/**
 * @ingroup qtWindow
 * @brief Main application with the QT_WidgetMain double thread class (X-Forms Thread)
 */
class Main_threads: public QObject
{

Q_OBJECT
#ifdef QT_GL
        qtGLWindow* 	g3dWin;
#endif
#ifdef QT_OPENGL_SIDE
        MainWidget* 	sideWin;
#endif
        QApplication* 	app;

public:
        Main_threads();
        ~Main_threads();

public:
        int run(int argc, char** argv);

private slots :
        void exit();

};

static char* molecule_xpm[] = {
	(char*) "32 32 147 2",
	(char*) "  	g #FFFFFF",
	(char*) ". 	g #F7F7F7",
	(char*) "+ 	g #EBEBEB",
	(char*) "@ 	g #E6E6E6",
	(char*) "# 	g #E5E5E5",
	(char*) "$ 	g #E9E9E9",
	(char*) "% 	g #F4F4F4",
	(char*) "& 	g #F3F3F3",
	(char*) "* 	g #E4E4E4",
	(char*) "= 	g #D3D3D3",
	(char*) "- 	g #8C8C8C",
	(char*) "; 	g #868686",
	(char*) "> 	g #C9C9C9",
	(char*) ", 	g #EEEEEE",
	(char*) "' 	g #FBFBFB",
	(char*) ") 	g #BBBBBB",
	(char*) "! 	g #101010",
	(char*) "~ 	g #000000",
	(char*) "{ 	g #060606",
	(char*) "] 	g #A1A1A1",
	(char*) "^ 	g #F6F6F6",
	(char*) "/ 	g #F2F2F2",
	(char*) "( 	g #4A4A4A",
	(char*) "_ 	g #232323",
	(char*) ": 	g #3F3F3F",
	(char*) "< 	g #161616",
	(char*) "[ 	g #EDEDED",
	(char*) "} 	g #F9F9F9",
	(char*) "| 	g #A2A2A2",
	(char*) "1 	g #030303",
	(char*) "2 	g #7E7E7E",
	(char*) "3 	g #FEFEFE",
	(char*) "4 	g #B2B2B2",
	(char*) "5 	g #2A2A2A",
	(char*) "6 	g #484848",
	(char*) "7 	g #9F9F9F",
	(char*) "8 	g #E7E7E7",
	(char*) "9 	g #F1F1F1",
	(char*) "0 	g #454545",
	(char*) "a 	g #CECECE",
	(char*) "b 	g #EFEFEF",
	(char*) "c 	g #FDFDFD",
	(char*) "d 	g #D6D6D6",
	(char*) "e 	g #898989",
	(char*) "f 	g #797979",
	(char*) "g 	g #B5B5B5",
	(char*) "h 	g #EAEAEA",
	(char*) "i 	g #2D2D2D",
	(char*) "j 	g #FCFCFC",
	(char*) "k 	g #171717",
	(char*) "l 	g #808080",
	(char*) "m 	g #E3E3E3",
	(char*) "n 	g #303030",
	(char*) "o 	g #F5F5F5",
	(char*) "p 	g #646464",
	(char*) "q 	g #090909",
	(char*) "r 	g #DFDFDF",
	(char*) "s 	g #D9D9D9",
	(char*) "t 	g #C3C3C3",
	(char*) "u 	g #575757",
	(char*) "v 	g #5E5E5E",
	(char*) "w 	g #050505",
	(char*) "x 	g #DDDDDD",
	(char*) "y 	g #ABABAB",
	(char*) "z 	g #787878",
	(char*) "A 	g #BEBEBE",
	(char*) "B 	g #686868",
	(char*) "C 	g #F0F0F0",
	(char*) "D 	g #8F8F8F",
	(char*) "E 	g #6D6D6D",
	(char*) "F 	g #5D5D5D",
	(char*) "G 	g #9B9B9B",
	(char*) "H 	g #E8E8E8",
	(char*) "I 	g #6E6E6E",
	(char*) "J 	g #A6A6A6",
	(char*) "K 	g #E2E2E2",
	(char*) "L 	g #717171",
	(char*) "M 	g #979797",
	(char*) "N 	g #E0E0E0",
	(char*) "O 	g #656565",
	(char*) "P 	g #D5D5D5",
	(char*) "Q 	g #757575",
	(char*) "R 	g #FAFAFA",
	(char*) "S 	g #555555",
	(char*) "T 	g #323232",
	(char*) "U 	g #D8D8D8",
	(char*) "V 	g #4D4D4D",
	(char*) "W 	g #DCDCDC",
	(char*) "X 	g #6C6C6C",
	(char*) "Y 	g #BCBCBC",
	(char*) "Z 	g #2F2F2F",
	(char*) "` 	g #595959",
	(char*) " .	g #C8C8C8",
	(char*) "..	g #DEDEDE",
	(char*) "+.	g #D1D1D1",
	(char*) "@.	g #9A9A9A",
	(char*) "#.	g #696969",
	(char*) "$.	g #E1E1E1",
	(char*) "%.	g #DBDBDB",
	(char*) "&.	g #7B7B7B",
	(char*) "*.	g #3C3C3C",
	(char*) "=.	g #1E1E1E",
	(char*) "-.	g #666666",
	(char*) ";.	g #5F5F5F",
	(char*) ">.	g #939393",
	(char*) ",.	g #C4C4C4",
	(char*) "'.	g #A5A5A5",
	(char*) ").	g #191919",
	(char*) "!.	g #838383",
	(char*) "~.	g #313131",
	(char*) "{.	g #878787",
	(char*) "].	g #5A5A5A",
	(char*) "^.	g #D4D4D4",
	(char*) "/.	g #BFBFBF",
	(char*) "(.	g #8D8D8D",
	(char*) "_.	g #6B6B6B",
	(char*) ":.	g #272727",
	(char*) "<.	g #1B1B1B",
	(char*) "[.	g #1C1C1C",
	(char*) "}.	g #DADADA",
	(char*) "|.	g #8A8A8A",
	(char*) "1.	g #0C0C0C",
	(char*) "2.	g #010101",
	(char*) "3.	g #0B0B0B",
	(char*) "4.	g #292929",
	(char*) "5.	g #3B3B3B",
	(char*) "6.	g #424242",
	(char*) "7.	g #545454",
	(char*) "8.	g #4F4F4F",
	(char*) "9.	g #343434",
	(char*) "0.	g #070707",
	(char*) "a.	g #A8A8A8",
	(char*) "b.	g #D0D0D0",
	(char*) "c.	g #414141",
	(char*) "d.	g #B0B0B0",
	(char*) "e.	g #777777",
	(char*) "f.	g #585858",
	(char*) "g.	g #747474",
	(char*) "h.	g #959595",
	(char*) "i.	g #B7B7B7",
	(char*) "j.	g #F8F8F8",
	(char*) "k.	g #949494",
	(char*) "l.	g #262626",
	(char*) "m.	g #ACACAC",
	(char*) "n.	g #0A0A0A",
	(char*) "o.	g #B3B3B3",
	(char*) "p.	g #8B8B8B",
	(char*) "                                . + @ # $ %                     ",
	(char*) "                              & * = - ; > # ,                   ",
	(char*) "                            ' # ) ! ~ ~ { ] # ^                 ",
	(char*) "                            / # ( ~ ~ ~ ~ _ @ ,                 ",
	(char*) "                            / # : ~ ~ ~ ~ < @ [                 ",
	(char*) "                            } # | 1 ~ ~ ~ 2 # &     3 3         ",
	(char*) "                              , # 4 5 6 7 # $ 3 . + # * 8 9 3   ",
	(char*) "                                [ # 0 a # b c % # d e f g # h 3 ",
	(char*) "                                b @ i # +   j # > k ~ ~ ~ l * / ",
	(char*) "                              c + m n # b   o # p ~ ~ ~ ~ q r h ",
	(char*) "                      c ^ 9 , , s t u m &   o # v ~ ~ ~ ~ w x h ",
	(char*) "                    o , , , b b d y z d b   } # A w ~ ~ ~ B # C ",
	(char*) "        3         & , , % '   ' # D e d @ % $ # B p E F G # H c ",
	(char*) "  c C 8 * @ [ ' ^ , , }   3 . [ x I J r 8 s K p L # # # # , j   ",
	(char*) "j 8 * J z M N # $ , .   ' b , m d O t # K P B Q # h R R 3       ",
	(char*) ", # S ~ ~ ~ T U s , 3 c b , ^ + r V = W * X ; * h c   3         ",
	(char*) "@ Y ~ ~ ~ ~ ~ e d * R . , ^ ^ H # Z s P ` 2 d 8 } [ # * @ b j   ",
	(char*) "# g ~ ~ ~ ~ ~ v  .s * ..W H # +.@.Z d #.- $.%.9 @ N M &.J # 8 j ",
	(char*) "+ * *.~ ~ ~ =.- -.u ;.>.,...'._ 5 w ).!.x N ..* ..~.~ ~ ~ S # [ ",
	(char*) "R @ r {.].z ^.K d d /.(._.u Z :.<.~ [.E K r }.K |.~ ~ ~ ~ ~ Y @ ",
	(char*) "  R + # # # $ . h r %.K # K [.1.2.3.4.5.6.7.8.6 9.~ ~ ~ ~ ~ g # ",
	(char*) "      c R j     j [ h h # $.B 0.< 4.4.a...%.# # d =.~ ~ ~ *.m + ",
	(char*) "                  c C h H d b.c.F -.d.# $ ^ . % # ^.e.f.; r @ R ",
	(char*) "                    3 % # %.g.h.N * # C 3       . $ * # * + R   ",
	(char*) "                3 b # # # i.c.* h j.c               j } c       ",
	(char*) "              3 + # '.` ].n > * R                               ",
	(char*) "              o # - 2.~ ~ 2.k.# ^                               ",
	(char*) "              b # l.~ ~ ~ ~ n # C                               ",
	(char*) "              b # T ~ ~ ~ ~ *.# C                               ",
	(char*) "              j.* m.n.~ ~ 1.o.# }                               ",
	(char*) "                b # a e p.b.# C                                 ",
	(char*) "                  o h # # h ^                                   "};

#endif // MAIN_HPP
