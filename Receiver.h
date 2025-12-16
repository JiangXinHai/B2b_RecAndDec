#ifndef RECEIVER_H
#define RECEIVER_H

#include <QObject>
#include <QByteArray>
#include <QVector>
#include <QMap>
#include <QFile>
#include <complex>
#include <cmath>
#include <cstdint>

class Receiver : public QObject
{
    Q_OBJECT
public:
    explicit Receiver(QObject *parent = nullptr);
    void init();
    void setDemodParam(float centerFreq = 1207.14e6, float sampleRate = 20.46e6, int iqBit = 8);

signals:
    void receiveLog(const QString &log);
    void frameReady(const QByteArray &frame, uint8_t prn, uint8_t msgType);

public slots:
    void onRawDataReceived(const QByteArray &rawData);

private:
    // 解调子模块
    QByteArray demodIQData(const QByteArray &iqData);
    void debugSpectrum(const QVector<std::complex<float>>& bb);
    QVector<std::complex<float>> IQToBaseband(const QByteArray &iqData);
    QVector<std::complex<float>> carrierSync(const QVector<std::complex<float>> &baseband);
    QVector<float> codeSync(const QVector<std::complex<float>> &carrierSynced);
    QByteArray bitSync(const QVector<float> &codeSynced);
                    //prn
    QVector<float> generateB2bCode(uint8_t prn);
    void initPrnCodeCache();             // 预生成所有PRN测距码的初始化函数
    bool verifyCode(uint8_t prn, const QVector<float>& code);
                    /* ============ 码同步盲搜核心 ============ */
    struct CodeSearchResult {
        float metric;
        int codePhase;
    };
    CodeSearchResult coherentCodeSearch(
        const QVector<float> &signal,
        const QVector<float> &code
    );


    // 帧解析子模块
    bool findSyncHeader(QByteArray &buffer);
    bool parseFrameHeader(const QByteArray &frame, uint8_t &prn, uint8_t &msgType);
    QByteArray ldpcDecode(const QByteArray &encodedData);
    void initLdpcHMatrix();
    uint32_t calculateCRC24Q(const QByteArray &data);
    bool verifyCRC(const QByteArray &rawFrame);

    // 辅助工具函数
    QByteArray symbolsToBits(const QVector<uint8_t> &symbols);
    uint8_t gf26Add(uint8_t a, uint8_t b);
    uint8_t gf26Mul(uint8_t a, uint8_t b);
    uint8_t gf26Inv(uint8_t a);
    QVector<float> truncateLLR(const QVector<float> &llr, int n);
    QVector<float> expandTruncatedLLR(const QVector<float> &truncated, int n, int fullSize);
    QVector<float> multiplyLLRByGF(const QVector<float> &llr, uint8_t gfVal, int fullSize);
    QVector<bool> byteArrayToBitStream(const QByteArray &byteArray);
    QByteArray bitStreamToByteArray(const QVector<bool> &bitStream);

private:
    // 解调参数
    float m_centerFreq = 1207.14e6;
    float m_sampleRate = 20.46e6;
    int m_iqBit = 8;
    const float m_codeRate = 10.23e6;    // 规范6.2
    const float m_symbolRate = 1000;     // 规范5.1
    uint8_t m_currentPrn = 0;            // 当前同步的PRN号
    int m_codeSyncOffset = 0;                // 码同步全局绝对偏移
    QMap<uint8_t, QVector<float>> m_prnCodeCache; // PRN测距码缓存
    QFile m_demodBinFile;       // 解调字节的bin文件句柄
    QString m_binFileName;      // 文件名（含路径）
    QByteArray m_binFileBuffer;   //文件缓存

    // 跨帧连续缓存
    QVector<std::complex<float>> m_continuousCarrierBuf; // 载波同步连续数据缓存
    QByteArray m_bitStreamCache;                         // 位同步比特流缓存

    // 帧解析参数
    QByteArray m_frameBuffer;
    QByteArray m_iqBuffer;
    const uint16_t m_syncHeader = 0xEB90; // 规范7.2.1
    const int m_encodedFrameLen = 125;    // 1000符号=125字节
    const int m_decodedFrameLen = 61;     // 486bit=61字节
    QVector<QVector<uint8_t>> m_ldpcH;    // 81×162 GF(2^6)校验矩阵

    // PRN→LFSR2初始值映射（规范表5完整数据）
    const QMap<uint8_t, QVector<int>> m_prnLfsr2Bits = {
        {6,  {1,0,0,0,1,1,0,1,0,1,1,1,0}},
        {7,  {1,0,0,0,1,1,1,1,0,1,1,1,0}},
        {8,  {1,0,0,0,1,1,1,1,1,1,0,1,1}},
        {9,  {1,0,0,1,1,0,0,1,0,1,0,0,1}},
        {10, {1,0,0,1,1,1,1,0,1,1,0,1,0}},
        {11, {1,0,1,0,0,0,0,1,1,0,1,0,1}},
        {12, {1,0,1,0,0,0,1,0,0,0,1,0,0}},
        {13, {1,0,1,0,0,0,1,0,1,0,1,0,1}},
        {14, {1,0,1,0,0,0,1,0,1,1,0,1,1}},
        {15, {1,0,1,0,0,0,1,0,1,1,1,0,0}},
        {16, {1,0,1,0,0,1,0,1,0,0,0,1,1}},
        {17, {1,0,1,0,0,1,1,1,1,0,1,1,1}},
        {18, {1,0,1,0,1,0,0,0,0,0,0,0,1}},
        {19, {1,0,1,0,1,0,0,1,1,1,1,1,0}},
        {20, {1,0,1,0,1,1,0,1,0,1,0,1,1}},
        {21, {1,0,1,0,1,1,0,1,1,0,0,0,1}},
        {22, {1,0,1,1,0,0,1,0,1,0,0,1,1}},
        {23, {1,0,1,1,0,0,1,1,0,0,0,1,0}},
        {24, {1,0,1,1,0,1,0,0,1,1,0,0,0}},
        {25, {1,0,1,1,0,1,0,1,1,0,1,1,0}},
        {26, {1,0,1,1,0,1,1,1,1,0,0,1,0}},
        {27, {1,0,1,1,0,1,1,1,1,1,1,1,1}},
        {28, {1,0,1,1,1,0,0,0,1,0,0,1,0}},
        {29, {1,0,1,1,1,0,0,1,1,1,1,0,0}},
        {30, {1,0,1,1,1,1,0,1,0,0,0,0,1}},
        {31, {1,0,1,1,1,1,1,0,0,1,0,0,0}},
        {32, {1,0,1,1,1,1,1,0,1,0,1,0,0}},
        {33, {1,0,1,1,1,1,1,1,0,1,0,1,1}},
        {34, {1,0,1,1,1,1,1,1,1,0,0,1,1}},
        {35, {1,1,0,0,0,0,1,0,1,0,0,0,1}},
        {36, {1,1,0,0,0,1,0,0,1,0,1,0,0}},
        {37, {1,1,0,0,0,1,0,1,1,0,1,1,1}},
        {38, {1,1,0,0,1,0,0,0,1,0,0,0,1}},
        {39, {1,1,0,0,1,0,0,0,1,1,0,0,1}},
        {40, {1,1,0,0,1,1,0,1,0,1,0,1,1}},
        {41, {1,1,0,0,1,1,0,1,1,0,0,0,1}},
        {42, {1,1,0,0,1,1,1,0,1,0,0,1,0}},
        {43, {1,1,0,1,0,0,1,0,1,0,1,0,1}},
        {44, {1,1,0,1,0,0,1,1,1,0,1,0,0}},
        {45, {1,1,0,1,0,1,1,0,0,1,0,1,1}},
        {46, {1,1,0,1,1,0,1,0,1,0,1,1,1}},
        {47, {1,1,1,0,0,0,0,1,1,0,1,0,0}},
        {48, {1,1,1,0,0,1,0,0,0,0,0,1,1}},
        {49, {1,1,1,0,0,1,0,0,0,1,0,1,1}},
        {50, {1,1,1,0,0,1,0,1,0,0,0,1,1}},
        {51, {1,1,1,0,0,1,0,1,0,1,0,0,0}},
        {52, {1,1,1,0,1,0,0,1,1,1,0,1,1}},
        {53, {1,1,1,0,1,1,0,0,1,0,1,1,1}},
        {54, {1,1,1,1,0,0,1,0,0,1,0,0,0}},
        {55, {1,1,1,1,0,1,0,0,1,0,1,0,0}},
        {56, {1,1,1,1,0,1,0,0,1,1,0,0,1}},
        {57, {1,1,1,1,0,1,1,0,1,1,0,1,0}},
        {58, {1,1,1,1,0,1,1,1,1,1,0,0,0}}
    };

    // GF(2^6)逆元表（规范附录A）
    const uint8_t m_gf26InvTable[64] = {
        0, 1, 32, 3, 16, 5, 34, 7, 8, 9, 36, 11, 18, 13, 38, 15,
        4, 17, 33, 19, 10, 37, 21, 39, 24, 25, 40, 27, 20, 41, 28, 12,
        22, 43, 26, 45, 42, 29, 47, 14, 49, 50, 52, 53, 55, 56, 57, 30,
        51, 54, 44, 46, 23, 48, 31, 58, 35, 60, 59, 62, 63, 61, 2, 3
    };
};

#endif // RECEIVER_H
