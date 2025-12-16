#include "Receiver.h"
#include <QDebug>
#include <algorithm>
#include <cmath>
#include <QDateTime>

Receiver::Receiver(QObject *parent) : QObject(parent)
{
    m_iqBuffer.reserve(81840);
    m_frameBuffer.reserve(2048);
    m_binFileBuffer.reserve(2048);

}

void Receiver::init()
{
    emit receiveLog("【init】开始执行显式初始化...");

    // 1. 初始化LDPC矩阵
    emit receiveLog("【init】初始化LDPC校验矩阵...");
    initLdpcHMatrix();

    /*        IQ解调，pass
    // 2. 预生成PRN码缓存（核心，带完整日志）
    emit receiveLog("【init】开始初始化PRN测距码缓存...");
    initPrnCodeCache();

    // 3. 初始化BIN文件（仅保留BIN文件逻辑）
    emit receiveLog("【init】初始化解调字节保存文件...");
    m_binFileName = QString("demod_bytes_%1.bin").arg(QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss"));
    m_demodBinFile.setFileName(m_binFileName);
    if (!m_demodBinFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
        emit receiveLog(QString("【init】打开二进制文件失败：%1").arg(m_demodBinFile.errorString()));
    } else {
        emit receiveLog(QString("【init】二进制文件已创建：%1").arg(m_binFileName));
    }
    */

    emit receiveLog("【init】显式初始化完成！");
}

void Receiver::setDemodParam(float centerFreq, float sampleRate, int iqBit)
{
    m_centerFreq = centerFreq;
    m_sampleRate = sampleRate;
    m_iqBit = iqBit;
    emit receiveLog(QString("解调参数配置完成：中心频=%1MHz，采样率=%2MHz，IQ位宽=%3bit").arg(centerFreq/1e6).arg(sampleRate/1e6).arg(iqBit));
}

void Receiver::initPrnCodeCache() {
    emit receiveLog("【PRN缓存】开始预生成所有PRN测距码（6~58）");
    // 遍历所有PRN，提前生成并缓存
    for (auto prn : m_prnLfsr2Bits.keys()) {
        QVector<float> b2bCode = generateB2bCode(prn);
        if (b2bCode.size() == 10230) { // 仅缓存有效码
            m_prnCodeCache[prn] = b2bCode;
            //emit receiveLog(QString("【PRN缓存】PRN=%1 测距码缓存完成").arg(prn));
        } else {
            emit receiveLog(QString("【PRN缓存】PRN=%1 测距码长度异常（%2），跳过缓存").arg(prn).arg(b2bCode.size()));
        }
    }
    emit receiveLog(QString("【PRN缓存】初始化完成，共缓存%1个PRN的测距码").arg(m_prnCodeCache.size()));
}

void Receiver::onRawDataReceived(const QByteArray &rawData)
{
    if (rawData.isEmpty()) {
        //emit receiveLog("接收空数据，忽略");
        return;
    }


// ----------------------------------------------------原始数据已经经过IQ解调-------------------------------------------
/*
    m_iqBuffer.append(rawData);
    emit receiveLog(QString("接收IQ采样数据%1字节，IQ缓冲区总长度：%2字节").arg(rawData.size()).arg(m_iqBuffer.size()));

//      单帧IQ长度：20460个复数点 × 2字节/点 = 40920字节（对应1个符号）
    const int iqFrameSize = static_cast<int>(m_sampleRate / m_symbolRate * 2);
    while (m_iqBuffer.size() >= iqFrameSize) {
        emit receiveLog("IQ缓存大小达标");
        QByteArray iqFrame = m_iqBuffer.left(iqFrameSize);
        m_iqBuffer.remove(0, iqFrameSize);

         // 解调：1帧IQ → 1个符号 → 1比特 → 累计8比特返回1字节（否则返回空）
        QByteArray singleByte  = demodIQData(iqFrame);
        if (singleByte .isEmpty()) {
            //emit receiveLog("IQ解调/位同步：暂未凑齐8比特，无字节输出");
            continue;
        }
        // 累加：1字节→125字节（1000比特）
        m_frameBuffer.append(singleByte);
        emit receiveLog(QString("单字节追加到帧缓存，当前帧缓存长度：%1字节（目标125字节）\n")
                        .arg(m_frameBuffer.size()));
        m_binFileBuffer.append(singleByte);
    }

    // 减少文件io次数
    if (m_demodBinFile.isOpen()) {
        // 直接写入原始二进制字节（无任何格式、无冗余）
        m_demodBinFile.write(m_binFileBuffer);
        m_demodBinFile.flush();
        //emit receiveLog(QString("【文件保存】已追加字节到BIN文件：%1").arg(m_binFileName));
        m_binFileBuffer.clear();
    }
*/
// ----------------------------------------------------------pass--------------------------------------------------------


    m_frameBuffer.append(rawData);
    while (m_frameBuffer.size() >= m_encodedFrameLen) {
        emit receiveLog("==================================================");
        emit receiveLog("编码帧长度达标，开始解码B-CNAV3帧");
        if (!findSyncHeader(m_frameBuffer)) {
            emit receiveLog("未找到同步头0xEB90，清空无效编码帧数据");
            m_frameBuffer.clear();
            break;
        }

        QByteArray encodedFrame = m_frameBuffer.left(m_encodedFrameLen);
        m_frameBuffer.remove(0, m_encodedFrameLen);

        QByteArray decodedFrame = ldpcDecode(encodedFrame.mid(2));
        if (decodedFrame.size() != m_decodedFrameLen) {
            emit receiveLog("LDPC译码失败，丢弃该帧");
            continue;
        }

        if (!verifyCRC(decodedFrame)) {
            emit receiveLog("CRC校验失败，丢弃该帧");
            continue;
        }

        uint8_t prn = 0;
        uint8_t msgType = 0;
        if (!parseFrameHeader(decodedFrame, prn, msgType)) {
            emit receiveLog("帧头解析失败，丢弃该帧");
            continue;
        }

        // 验证解帧PRN与码同步PRN一致性
        if (m_currentPrn != 0 && m_currentPrn != prn) {
            emit receiveLog(QString("PRN不一致（码同步=%1，解帧=%2），重新同步").arg(m_currentPrn).arg(prn));
            m_currentPrn = 0;
            continue;
        }

        emit frameReady(decodedFrame, prn, msgType);
        emit receiveLog(QString("解析到有效B-CNAV3帧：PRN=%1，信息类型=%2，译码后长度=%3字节").arg(prn).arg(msgType).arg(decodedFrame.size()));
    }
}


//IQ解调，不再使用
/***
QByteArray Receiver::demodIQData(const QByteArray &iqData)
{
    //emit receiveLog("开始IQ解调");
    QVector<std::complex<float>> baseband = IQToBaseband(iqData);
    if (baseband.isEmpty()) return QByteArray();

    debugSpectrum(baseband);

    QVector<std::complex<float>> carrierSynced = carrierSync(baseband);
    if (carrierSynced.isEmpty()) return QByteArray();

    QVector<float> codeSynced = codeSync(carrierSynced);
    if (codeSynced.isEmpty()) return QByteArray();

    return bitSync(codeSynced);
}

void Receiver::debugSpectrum(const QVector<std::complex<float>>& bb)
{
    const int N = 2048;
    if (bb.size() < N) return;

    float power = 0.0f;
    for (int i = 0; i < N; ++i)
        power += std::norm(bb[i]);

    emit receiveLog(QString("【DEBUG】平均功率=%1").arg(power / N));
}

QVector<std::complex<float>> Receiver::IQToBaseband(const QByteArray &iqData)
{
    //emit receiveLog(QString("【IQ转基带】开始处理，输入IQ数据长度：%1字节").arg(iqData.size()));
    QVector<std::complex<float>> bb;

    if (iqData.size() % 2 != 0) {
        emit receiveLog("【IQ转基带】IQ 数据长度错误");
        return bb;
    }

    bb.reserve(iqData.size() / 2);

    for (int i = 0; i < iqData.size(); i += 2) {

        // 用 int8_t，不是 uint8_t
        float I = static_cast<int8_t>(iqData[i])     / 128.0f;
        float Q = static_cast<int8_t>(iqData[i + 1]) / 128.0f;

        bb.append(std::complex<float>(I, Q));
    }

    emit receiveLog(
        QString("【IQ转基带】完成，输出 %1 点").arg(bb.size())
    );
    return bb;
}

QVector<std::complex<float>> Receiver::carrierSync(const QVector<std::complex<float>> &baseband)
{
    // 1. 开始同步日志（标注输入长度）
    //emit receiveLog(QString("【载波同步】开始处理，输入基带数据长度：%1个复数点").arg(baseband.size()));
    QVector<std::complex<float>> synced;
    synced.reserve(baseband.size());

    float phase = 0.0f;
    float freq  = 0.0f;

    const float kp = 0.02f;
    const float ki = 0.0002f;

    for (auto s : baseband) {

        std::complex<float> rot =
            s * std::complex<float>(std::cos(-phase), std::sin(-phase));

        synced.append(rot);
        m_continuousCarrierBuf.append(rot);

        float I = rot.real();
        float Q = rot.imag();

        float error = I * Q;

        freq  += ki * error;
        phase += freq + kp * error;

        if (phase > M_PI)  phase -= 2*M_PI;
        if (phase < -M_PI) phase += 2*M_PI;
    }

    return synced;
}

QVector<float> Receiver::codeSync(const QVector<std::complex<float>> &carrierSynced)
{
    const int samplesPerCode = 10230 * 2;
    if (m_continuousCarrierBuf.size() < samplesPerCode * 3)
        return {};

    QVector<float> signal;
    signal.reserve(20460);

    for (int i = 0; i < samplesPerCode * 3; ++i)
        signal.append(m_continuousCarrierBuf[i].real());

    if (m_currentPrn == 0) {

        float bestMetric = 0.0f;
        uint8_t bestPrn = 0;
        int bestPhase = 0;

        for (auto prn : m_prnCodeCache.keys()) {

            auto &code = m_prnCodeCache[prn];
            auto res = coherentCodeSearch(signal, code);

            emit receiveLog(QString("PRN=%1 metric=%2")
                .arg(prn).arg(res.metric, 0, 'f', 4));

            if (res.metric > bestMetric) {
                bestMetric = res.metric;
                bestPrn = prn;
                bestPhase = res.codePhase;
            }
        }

        if (bestMetric < 0.2f) {   //  阈值应明显更高
            emit receiveLog("【码同步】盲搜失败");
            return {};
        }

        m_currentPrn = bestPrn;
        m_codeSyncOffset = bestPhase;

        emit receiveLog(QString("【码同步】锁定 PRN=%1 metric=%2")
            .arg(bestPrn).arg(bestMetric, 0, 'f', 4));
    }

    // 解扩
    QVector<float> despread;
    auto &code = m_prnCodeCache[m_currentPrn];

    for (int i = 0; i < 10230; ++i) {
        int idx = m_codeSyncOffset + i * 2;
        despread.append(signal[idx] * code[i]);
    }

    m_continuousCarrierBuf.remove(0, 20460);
    return despread;
}

QByteArray Receiver::bitSync(const QVector<float> &despread)
{
    float sum = 0.0f;
    for (float v : despread)
        sum += v;

    uint8_t bit = (sum > 0) ? 0 : 1;
    m_bitStreamCache.append(bit);

    QByteArray out;
    if (m_bitStreamCache.size() >= 8) {
        uint8_t byte = 0;
        for (int i = 0; i < 8; ++i)
            byte = (byte << 1) | m_bitStreamCache[i];

        m_bitStreamCache.remove(0, 8);
        out.append(byte);
    }
    return out;
}

Receiver::CodeSearchResult Receiver::coherentCodeSearch(
    const QVector<float> &signal,
    const QVector<float> &code)
{
    const int samplesPerChip = 2;
    const int codeLen = 10230;
    const int samplesPerCode = codeLen * samplesPerChip;

    if (signal.size() < samplesPerCode * 3)
        return {0.0f, 0};

    float bestMetric = 0.0f;
    int bestPhase = 0;

    //  只搜索 chip 相位
    for (int phase = 0; phase < samplesPerChip; ++phase) {

        float accEnergy = 0.0f;
        float noiseEnergy = 0.0f;

        //  non-coherent 累加多个码周期
        for (int blk = 0; blk < 3; ++blk) {

            float acc = 0.0f;

            int base = blk * samplesPerCode;

            for (int i = 0; i < codeLen; ++i) {
                int idx = base + phase + i * samplesPerChip;
                float v = signal[idx] * code[i];
                acc += v;
                noiseEnergy += std::abs(v);
            }

            accEnergy += std::abs(acc);
        }

        float metric = accEnergy / (noiseEnergy + 1e-6f);
        if (metric > bestMetric) {
            bestMetric = metric;
            bestPhase = phase;
        }
    }

    return {bestMetric, bestPhase};
}

QVector<float> Receiver::generateB2bCode(uint8_t prn)
{
    const int CODE_LEN = 10230;
    QVector<float> code;
    code.reserve(CODE_LEN);

    uint16_t lfsr1 = 0x1FFF;   // s1,1~s1,13 全为 1

    uint16_t lfsr2 = 0;
    const QVector<int>& initBits = m_prnLfsr2Bits.value(prn); // [s2,1 ... s2,13]

    for (int i = 0; i < 13; ++i) {
        if (initBits[i]) {
            lfsr2 |= (1u << (12 - i));  // s2,1 → bit12
        }
    }

    for (int i = 0; i < CODE_LEN; ++i) {

        uint8_t s1_13 = lfsr1 & 0x01;   // bit0
        uint8_t s2_13 = lfsr2 & 0x01;   // bit0
        uint8_t chip  = s1_13 ^ s2_13;

        code.append(chip ? -1.0f : 1.0f);

        uint8_t fb1 =
              ((lfsr1 >> 0 ) & 0x01)
            ^ ((lfsr1 >> 3 ) & 0x01)
            ^ ((lfsr1 >> 4 ) & 0x01)
            ^ ((lfsr1 >> 12) & 0x01);

        uint8_t fb2 =
              ((lfsr2 >> 0 ) & 0x01)
            ^ ((lfsr2 >> 1 ) & 0x01)
            ^ ((lfsr2 >> 4 ) & 0x01)
            ^ ((lfsr2 >> 7 ) & 0x01)
            ^ ((lfsr2 >> 9 ) & 0x01)
            ^ ((lfsr2 >> 10) & 0x01);

        lfsr1 = (lfsr1 >> 1) | (uint16_t(fb1) << 12);
        lfsr2 = (lfsr2 >> 1) | (uint16_t(fb2) << 12);

        lfsr1 &= 0x1FFF;
        lfsr2 &= 0x1FFF;

        if (i == 8189) {
            lfsr1 = 0x1FFF;
        }
    }

    if (!verifyCode(prn, code)) {
        code.clear();
    }

    return code;
}

// 验证逻辑：比对首尾24个码片与规范表5
bool Receiver::verifyCode(uint8_t prn, const QVector<float>& code)
{
    if (code.size() != 10230) return false;

    // 定义PRN对应的标准首尾码片
    QMap<uint8_t, QPair<QString, QString>> prnCodeStd = {
        {6,  {"42471422", "44530033"}},
        {7,  {"42071026", "63454537"}},
        {8,  {"10070621", "52114120"}},
        {9,  {"32631660", "15654621"}},
        {10, {"51031210", "12615765"}},
        {11, {"24752203", "23740542"}},//
        {12, {"67353533", "07467654"}},
        {13, {"25353617", "52575257"}},//
        {14, {"11351722", "55226274"}},
        {15, {"61351343", "01160270"}},
        {16, {"16550441", "50756326"}},
        {17, {"04153547", "27542214"}},//
        {18, {"37651752", "10640254"}},
        {19, {"40652553", "14350465"}},
        {20, {"12451253", "57452211"}},
        {21, {"34450664", "00071604"}},
        {22, {"15313657", "10263607"}},
        {23, {"56312563", "13020015"}},
        {24, {"71510447", "47474176"}},
        {25, {"44513562", "16076344"}},
        {26, {"54112445", "55540654"}},
        {27, {"00111432", "62507667"}},
        {28, {"55610115", "63416213"}},
        {29, {"60613030", "32014021"}},
        {30, {"36410161", "43533653"}},
        {31, {"73013021", "61313161"}},
        {32, {"65010372", "03246551"}},
        {33, {"12013173", "07756360"}},
        {34, {"14011703", "01251744"}},
        {35, {"35360744", "27367153"}},
        {36, {"65561461", "77223601"}},
        {37, {"04561533", "11666400"}},
        {38, {"35661303", "35322566"}},
        {39, {"31661552", "07107560"}},
        {40, {"12463623", "46612101"}},//
        {41, {"34462214", "11231514"}},
        {42, {"55062742", "50710211"}},
        {43, {"25323543", "34555532"}},
        {44, {"64320656", "03034702"}},
        {45, {"13121550", "75766350"}},
        {46, {"05221747", "50550432"}},//
        {47, {"64741521", "45030464"}},
        {48, {"17540076", "01547030"}},
        {49, {"13540627", "33762036"}},
        {50, {"16541066", "57616221"}},
        {51, {"72540775", "55327237"}},
        {52, {"10640752", "16072557"}},
        {53, {"05442537", "64716537"}},
        {54, {"73301542", "21130334"}},
        {55, {"65500312", "16343063"}},
        {56, {"31503365", "21304050"}},
        {57, {"51102623", "36574544"}},
        {58, {"70100474", "31701764"}}
    };

    if (!prnCodeStd.contains(prn)) return true; // 无标准值则跳过验证

    auto codeToOct = [](const QVector<float>& code, int start, int len) -> QString {
        QString binary;
        for (int i = 0; i < len; ++i) {
            // 1.0f→逻辑0→二进制0；-1.0f→逻辑1→二进制1（规范5.3）
            binary += (code[start + i] == 1.0f) ? "0" : "1";
        }
        // 高位先传：二进制字符串左侧是高位，不足24位补0到左侧
        binary = binary.leftJustified(24, '0', true); // 补0到24位，左侧补

        // 二进制转八进制：从左到右每3位分组（高位→低位）
        QString oct;
        for (int i = 0; i < binary.size(); i += 3) {
            QString sub = binary.mid(i, 3);
            // 避免空字符串，确保3位
            if (sub.size() < 3) sub = sub.rightJustified(3, '0');
            oct += QString::number(sub.toInt(nullptr, 2), 8);
        }
        return oct;
    };

    QString headOct = codeToOct(code, 0, 24);
    QString tailOct = codeToOct(code, 10230 - 24, 24);
    QPair<QString, QString> std = prnCodeStd[prn];

    if (headOct == std.first && tailOct == std.second) {
        emit receiveLog(QString("PRN=%1 码片验证通过：头24=%2，末尾24=%3").arg(prn).arg(headOct).arg(tailOct));
        return true;
    } else {
        emit receiveLog(QString("PRN=%1 码片验证失败！").arg(prn));
        emit receiveLog(QString("  实际头24：%1，标准头24：%2").arg(headOct).arg(std.first));
        emit receiveLog(QString("  实际末尾24：%1，标准末尾24：%2").arg(tailOct).arg(std.second));
        return false;
    }
}
*/

void Receiver::initLdpcHMatrix()
{
    // 初始化81×162全0矩阵
    m_ldpcH.resize(81);
    for (int i = 0; i < 81; ++i) {
        m_ldpcH[i].resize(162);
        std::fill(m_ldpcH[i].begin(), m_ldpcH[i].end(), 0);
    }

    // ========== 第0行 ==========
    m_ldpcH[0][23] = 46; m_ldpcH[0][109] = 15; m_ldpcH[0][130] = 53;
    m_ldpcH[0][120] = 15; m_ldpcH[0][83] = 31; m_ldpcH[0][125] = 61;

    // ========== 第1行 ==========
    m_ldpcH[1][79] = 54; m_ldpcH[1][104] = 38; m_ldpcH[1][132] = 51;
    m_ldpcH[1][138] = 59; m_ldpcH[1][71] = 63; m_ldpcH[1][111] = 56;
    m_ldpcH[1][127] = 13;

    // ========== 第2行 ==========
    m_ldpcH[2][42] = 26; m_ldpcH[2][101] = 22; m_ldpcH[2][146] = 14;
    m_ldpcH[2][18] = 2; m_ldpcH[2][66] = 63; m_ldpcH[2][108] = 26;
    m_ldpcH[2][160] = 41;

    // ========== 第3行 ==========
    m_ldpcH[3][61] = 35; m_ldpcH[3][113] = 31; m_ldpcH[3][126] = 44;
    m_ldpcH[3][8] = 44; m_ldpcH[3][50] = 51; m_ldpcH[3][89] = 35;
    m_ldpcH[3][131] = 13; m_ldpcH[3][34] = 30; m_ldpcH[3][157] = 1;
    m_ldpcH[3][100] = 44; m_ldpcH[3][145] = 7;

    // ========== 第4行 ==========
    m_ldpcH[4][60] = 16; m_ldpcH[4][112] = 63; m_ldpcH[4][128] = 20;
    m_ldpcH[4][0] = 9; m_ldpcH[4][49] = 27; m_ldpcH[4][115] = 56;
    m_ldpcH[4][151] = 8; m_ldpcH[4][6] = 43; m_ldpcH[4][106] = 1;
    m_ldpcH[4][144] = 44; m_ldpcH[4][33] = 30; m_ldpcH[4][53] = 24;
    m_ldpcH[4][82] = 5; m_ldpcH[4][140] = 26;

    // ========== 第5行 ==========
    uint8_t row5Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row5Pos[] = {3,45,84,126,38,80,109,147,9,60,96,141,1,43,82,124};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[5][row5Pos[idx]] = row5Vals[idx];
    }

    // ========== 第6行 ==========
    uint8_t row6Vals[] = {35,31,44,44,51,35,13,30,1,44,7,27,5,2,62};
    int row6Pos[] = {20,77,88,158,37,54,122,159,3,65,104,149,5,47,128};
    for (int idx = 0; idx < 15; ++idx) {
        m_ldpcH[6][row6Pos[idx]] = row6Vals[idx];
    }

    // ========== 第7行 ==========
    uint8_t row7Vals[] = {16,63,20,9,27,56,8,43,1,44,30,24,5,26,27,37};
    int row7Pos[] = {0,42,81,123,32,79,97,120,35,72,112,158,15,57,93,138};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[7][row7Pos[idx]] = row7Vals[idx];
    }

    // ========== 第8行 ==========
    uint8_t row8Vals[] = {35,31,44,44,51,35,13,30,1,44,7,27,5,2,62};
    int row8Pos[] = {22,75,107,143,24,69,102,133,1,50,116,152,24,57,119,135};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[8][row8Pos[idx]] = row8Vals[idx];
    }

    // ========== 第9行 ==========
    uint8_t row9Vals[] = {16,63,20,9,27,56,8,43,1,44,30,24,5,26,27,37};
    int row9Pos[] = {17,59,95,140,7,45,107,145,34,51,83,138,14,43,99,144};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[9][row9Pos[idx]] = row9Vals[idx];
    }

    // ========== 第10行 ==========
    uint8_t row10Vals[] = {60,24,4,50,32,49,58,19,43,34,48,57,29,7,10,16};
    int row10Pos[] = {21,77,106,142,16,58,94,139,20,68,110,131,2,48,114,150};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[10][row10Pos[idx]] = row10Vals[idx];
    }

    // ========== 第11行 ==========
    uint8_t row11Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row11Pos[] = {10,52,91,133,25,70,103,134,32,41,95,153,14,56,91,137};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[11][row11Pos[idx]] = row11Vals[idx];
    }

    // ========== 第12行 ==========
    uint8_t row12Vals[] = {46,15,53,15,31,61,22,14,2,63,26,41,12,17,32,58};
    int row12Pos[] = {1,48,87,129,40,81,110,148,10,61,97,142,2,44,83,125};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[12][row12Pos[idx]] = row12Vals[idx];
    }

    // ========== 第13行 ==========
    uint8_t row13Vals[] = {54,38,51,59,63,56,13,26,22,14,2,63,26,41,12,17};
    int row13Pos[] = {11,53,92,134,26,71,104,135,33,42,96,143,15,57,92,138};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[13][row13Pos[idx]] = row13Vals[idx];
    }

    // ========== 第14行 ==========
    uint8_t row14Vals[] = {35,31,44,44,51,35,13,30,1,44,7,27,5,2,62,16};
    int row14Pos[] = {18,60,98,145,8,51,89,132,4,66,105,150,16,59,94,141};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[14][row14Pos[idx]] = row14Vals[idx];
    }

    // ========== 第15行 ==========
    uint8_t row15Vals[] = {63,20,9,27,56,8,43,1,44,30,24,5,26,27,37,46};
    int row15Pos[] = {19,61,99,146,9,52,90,133,5,67,106,151,17,60,95,142};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[15][row15Pos[idx]] = row15Vals[idx];
    }

    // ========== 第16行 ==========
    uint8_t row16Vals[] = {32,49,58,19,43,34,48,57,29,7,10,16,46,15,53,15};
    int row16Pos[] = {20,62,100,147,10,53,91,134,6,68,107,152,18,61,96,143};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[16][row16Pos[idx]] = row16Vals[idx];
    }

    // ========== 第17行 ==========
    uint8_t row17Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row17Pos[] = {21,63,101,148,11,54,92,135,7,69,108,153,19,62,97,144};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[17][row17Pos[idx]] = row17Vals[idx];
    }

    // ========== 第18行 ==========
    uint8_t row18Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row18Pos[] = {22,64,102,149,12,55,93,136,8,70,109,154,20,63,98,145};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[18][row18Pos[idx]] = row18Vals[idx];
    }

    // ========== 第19行 ==========
    uint8_t row19Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row19Pos[] = {23,65,103,150,13,56,94,137,9,71,110,155,21,64,99,146};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[19][row19Pos[idx]] = row19Vals[idx];
    }

    // ========== 第20行 ==========
    uint8_t row20Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row20Pos[] = {24,66,104,151,14,57,95,138,10,72,111,156,22,65,100,147};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[20][row20Pos[idx]] = row20Vals[idx];
    }

    // ========== 第21行 ==========
    uint8_t row21Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row21Pos[] = {25,67,105,152,15,58,96,139,11,73,112,157,23,66,101,148};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[21][row21Pos[idx]] = row21Vals[idx];
    }

    // ========== 第22行 ==========
    uint8_t row22Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row22Pos[] = {26,68,106,153,16,59,97,140,12,74,113,158,24,67,102,149};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[22][row22Pos[idx]] = row22Vals[idx];
    }

    // ========== 第23行 ==========
    uint8_t row23Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row23Pos[] = {27,69,107,154,17,60,98,141,13,75,114,159,25,68,103,150};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[23][row23Pos[idx]] = row23Vals[idx];
    }

    // ========== 第24行 ==========
    uint8_t row24Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row24Pos[] = {28,70,108,155,18,61,99,142,14,76,115,160,26,69,104,151};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[24][row24Pos[idx]] = row24Vals[idx];
    }

    // ========== 第25行 ==========
    uint8_t row25Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row25Pos[] = {29,71,109,156,19,62,100,143,15,77,116,161,27,70,105,152};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[25][row25Pos[idx]] = row25Vals[idx];
    }

    // ========== 第26行 ==========
    uint8_t row26Vals[] = {58,37,38,23,55,22,46,15,53,15,31,61,26,22,14,2};
    int row26Pos[] = {30,72,110,157,20,63,101,144,16,78,117,161,28,71,106,153};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[26][row26Pos[idx]] = row26Vals[idx];
    }

    // ========== 第27行 ==========
    uint8_t row27Vals[] = {63,26,41,12,17,32,58,37,38,23,55,22,46,15,53,15};
    int row27Pos[] = {31,73,111,158,21,64,102,145,17,79,118,0,29,72,107,154};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[27][row27Pos[idx]] = row27Vals[idx];
    }

    // ========== 第28行 ==========
    uint8_t row28Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row28Pos[] = {32,74,112,159,22,65,103,146,18,80,119,1,30,73,108,155};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[28][row28Pos[idx]] = row28Vals[idx];
    }

    // ========== 第29行 ==========
    uint8_t row29Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row29Pos[] = {33,75,113,160,23,66,104,147,19,81,120,2,31,74,109,156};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[29][row29Pos[idx]] = row29Vals[idx];
    }

    // ========== 第30行 ==========
    uint8_t row30Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row30Pos[] = {34,76,114,0,24,67,105,148,20,82,121,3,32,75,110,157};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[30][row30Pos[idx]] = row30Vals[idx];
    }

    // ========== 第31行 ==========
    uint8_t row31Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row31Pos[] = {35,77,115,1,25,68,106,149,21,83,122,4,33,76,111,158};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[31][row31Pos[idx]] = row31Vals[idx];
    }

    // ========== 第32行 ==========
    uint8_t row32Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row32Pos[] = {36,78,116,2,26,69,107,150,22,84,123,5,34,77,112,159};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[32][row32Pos[idx]] = row32Vals[idx];
    }

    // ========== 第33行 ==========
    uint8_t row33Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row33Pos[] = {37,79,117,3,27,70,108,151,23,85,124,6,35,78,113,160};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[33][row33Pos[idx]] = row33Vals[idx];
    }

    // ========== 第34行 ==========
    uint8_t row34Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row34Pos[] = {38,80,118,4,28,71,109,152,24,86,125,7,36,79,114,0};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[34][row34Pos[idx]] = row34Vals[idx];
    }

    // ========== 第35行 ==========
    uint8_t row35Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row35Pos[] = {39,81,119,5,29,72,110,153,25,87,126,8,37,80,115,1};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[35][row35Pos[idx]] = row35Vals[idx];
    }

    // ========== 第36行 ==========
    uint8_t row36Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row36Pos[] = {40,82,120,6,30,73,111,154,26,88,127,9,38,81,116,2};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[36][row36Pos[idx]] = row36Vals[idx];
    }

    // ========== 第37行 ==========
    uint8_t row37Vals[] = {58,37,38,23,55,22,46,15,53,15,31,61,26,22,14,2};
    int row37Pos[] = {41,83,121,7,31,74,112,155,27,89,128,10,39,82,117,3};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[37][row37Pos[idx]] = row37Vals[idx];
    }

    // ========== 第38行 ==========
    uint8_t row38Vals[] = {63,26,41,12,17,32,58,37,38,23,55,22,46,15,53,15};
    int row38Pos[] = {42,84,122,8,32,75,113,156,28,90,129,11,40,83,118,4};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[38][row38Pos[idx]] = row38Vals[idx];
    }

    // ========== 第39行 ==========
    uint8_t row39Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row39Pos[] = {43,85,123,9,33,76,114,157,29,91,130,12,41,84,119,5};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[39][row39Pos[idx]] = row39Vals[idx];
    }

    // ========== 第40行 ==========
    uint8_t row40Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row40Pos[] = {44,86,124,10,34,77,115,158,30,92,131,13,42,85,120,6};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[40][row40Pos[idx]] = row40Vals[idx];
    }

    // ========== 第41行 ==========
    uint8_t row41Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row41Pos[] = {45,87,125,11,35,78,116,159,31,93,132,14,43,86,121,7};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[41][row41Pos[idx]] = row41Vals[idx];
    }

    // ========== 第42行 ==========
    uint8_t row42Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row42Pos[] = {46,88,126,12,36,79,117,160,32,94,133,15,44,87,122,8};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[42][row42Pos[idx]] = row42Vals[idx];
    }

    // ========== 第43行 ==========
    uint8_t row43Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row43Pos[] = {47,89,127,13,37,80,118,0,33,95,134,16,45,88,123,9};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[43][row43Pos[idx]] = row43Vals[idx];
    }

    // ========== 第44行 ==========
    uint8_t row44Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row44Pos[] = {48,90,128,14,38,81,119,1,34,96,135,17,46,89,124,10};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[44][row44Pos[idx]] = row44Vals[idx];
    }

    // ========== 第45行 ==========
    uint8_t row45Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row45Pos[] = {49,91,129,15,39,82,120,2,35,97,136,18,47,90,125,11};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[45][row45Pos[idx]] = row45Vals[idx];
    }

    // ========== 第46行 ==========
    uint8_t row46Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row46Pos[] = {50,92,130,16,40,83,121,3,36,98,137,19,48,91,126,12};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[46][row46Pos[idx]] = row46Vals[idx];
    }

    // ========== 第47行 ==========
    uint8_t row47Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row47Pos[] = {51,93,131,17,41,84,122,4,37,99,138,20,49,92,127,13};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[47][row47Pos[idx]] = row47Vals[idx];
    }

    // ========== 第48行 ==========
    uint8_t row48Vals[] = {58,37,38,23,55,22,46,15,53,15,31,61,26,22,14,2};
    int row48Pos[] = {52,94,132,18,42,85,123,5,38,100,139,21,50,93,128,14};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[48][row48Pos[idx]] = row48Vals[idx];
    }

    // ========== 第49行 ==========
    uint8_t row49Vals[] = {63,26,41,12,17,32,58,37,38,23,55,22,46,15,53,15};
    int row49Pos[] = {53,95,133,19,43,86,124,6,39,101,140,22,51,94,129,15};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[49][row49Pos[idx]] = row49Vals[idx];
    }

    // ========== 第50行 ==========
    uint8_t row50Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row50Pos[] = {54,96,134,20,44,87,125,7,40,102,141,23,52,95,130,16};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[50][row50Pos[idx]] = row50Vals[idx];
    }

    // ========== 第51行 ==========
    uint8_t row51Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row51Pos[] = {55,97,135,21,45,88,126,8,41,103,142,24,53,96,131,17};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[51][row51Pos[idx]] = row51Vals[idx];
    }

    // ========== 第52行 ==========
    uint8_t row52Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row52Pos[] = {56,98,136,22,46,89,127,9,42,104,143,25,54,97,132,18};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[52][row52Pos[idx]] = row52Vals[idx];
    }

    // ========== 第53行 ==========
    uint8_t row53Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row53Pos[] = {57,99,137,23,47,90,128,10,43,105,144,26,55,98,133,19};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[53][row53Pos[idx]] = row53Vals[idx];
    }

    // ========== 第54行 ==========
    uint8_t row54Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row54Pos[] = {58,100,138,24,48,91,129,11,44,106,145,27,56,99,134,20};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[54][row54Pos[idx]] = row54Vals[idx];
    }

    // ========== 第55行 ==========
    uint8_t row55Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row55Pos[] = {59,101,139,25,49,92,130,12,45,107,146,28,57,100,135,21};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[55][row55Pos[idx]] = row55Vals[idx];
    }

    // ========== 第56行 ==========
    uint8_t row56Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row56Pos[] = {60,102,140,26,50,93,131,13,46,108,147,29,58,101,136,22};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[56][row56Pos[idx]] = row56Vals[idx];
    }

    // ========== 第57行 ==========
    uint8_t row57Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row57Pos[] = {61,103,141,27,51,94,132,14,47,109,148,30,59,102,137,23};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[57][row57Pos[idx]] = row57Vals[idx];
    }

    // ========== 第58行 ==========
    uint8_t row58Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row58Pos[] = {62,104,142,28,52,95,133,15,48,110,149,31,60,103,138,24};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[58][row58Pos[idx]] = row58Vals[idx];
    }

    // ========== 第59行 ==========
    uint8_t row59Vals[] = {58,37,38,23,55,22,46,15,53,15,31,61,26,22,14,2};
    int row59Pos[] = {63,105,143,29,53,96,134,16,49,111,150,32,61,104,139,25};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[59][row59Pos[idx]] = row59Vals[idx];
    }

    // ========== 第60行 ==========
    uint8_t row60Vals[] = {63,26,41,12,17,32,58,37,38,23,55,22,46,15,53,15};
    int row60Pos[] = {64,106,144,30,54,97,135,17,50,112,151,33,62,105,140,26};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[60][row60Pos[idx]] = row60Vals[idx];
    }

    // ========== 第61行 ==========
    uint8_t row61Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row61Pos[] = {65,107,145,31,55,98,136,18,51,113,152,34,63,106,141,27};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[61][row61Pos[idx]] = row61Vals[idx];
    }

    // ========== 第62行 ==========
    uint8_t row62Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row62Pos[] = {66,108,146,32,56,99,137,19,52,114,153,35,64,107,142,28};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[62][row62Pos[idx]] = row62Vals[idx];
    }

    // ========== 第63行 ==========
    uint8_t row63Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row63Pos[] = {67,109,147,33,57,100,138,20,53,115,154,36,65,108,143,29};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[63][row63Pos[idx]] = row63Vals[idx];
    }

    // ========== 第64行 ==========
    uint8_t row64Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row64Pos[] = {68,110,148,34,58,101,139,21,54,116,155,37,66,109,144,30};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[64][row64Pos[idx]] = row64Vals[idx];
    }

    // ========== 第65行 ==========
    uint8_t row65Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row65Pos[] = {69,111,149,35,59,102,140,22,55,117,156,38,67,110,145,31};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[65][row65Pos[idx]] = row65Vals[idx];
    }

    // ========== 第66行 ==========
    uint8_t row66Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row66Pos[] = {70,112,150,36,60,103,141,23,56,118,157,39,68,111,146,32};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[66][row66Pos[idx]] = row66Vals[idx];
    }

    // ========== 第67行 ==========
    uint8_t row67Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row67Pos[] = {71,113,151,37,61,104,142,24,57,119,158,40,69,112,147,33};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[67][row67Pos[idx]] = row67Vals[idx];
    }

    // ========== 第68行 ==========
    uint8_t row68Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row68Pos[] = {72,114,152,38,62,105,143,25,58,120,159,41,70,113,148,34};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[68][row68Pos[idx]] = row68Vals[idx];
    }

    // ========== 第69行 ==========
    uint8_t row69Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row69Pos[] = {73,115,153,39,63,106,144,26,59,121,160,42,71,114,149,35};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[69][row69Pos[idx]] = row69Vals[idx];
    }

    // ========== 第70行 ==========
    uint8_t row70Vals[] = {58,37,38,23,55,22,46,15,53,15,31,61,26,22,14,2};
    int row70Pos[] = {74,116,154,40,64,107,145,27,60,122,0,43,72,115,150,36};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[70][row70Pos[idx]] = row70Vals[idx];
    }

    // ========== 第71行 ==========
    uint8_t row71Vals[] = {63,26,41,12,17,32,58,37,38,23,55,22,46,15,53,15};
    int row71Pos[] = {75,117,155,41,65,108,146,28,61,123,1,44,73,116,151,37};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[71][row71Pos[idx]] = row71Vals[idx];
    }

    // ========== 第72行 ==========
    uint8_t row72Vals[] = {31,61,26,22,14,2,63,26,41,12,17,32,58,37,38,23};
    int row72Pos[] = {76,118,156,42,66,109,147,29,62,124,2,45,74,117,152,38};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[72][row72Pos[idx]] = row72Vals[idx];
    }

    // ========== 第73行 ==========
    uint8_t row73Vals[] = {55,22,46,15,53,15,31,61,26,22,14,2,63,26,41,12};
    int row73Pos[] = {77,119,157,43,67,110,148,30,63,125,3,46,75,118,153,39};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[73][row73Pos[idx]] = row73Vals[idx];
    }

    // ========== 第74行 ==========
    uint8_t row74Vals[] = {17,32,58,37,38,23,55,22,46,15,53,15,31,61,26,22};
    int row74Pos[] = {78,120,158,44,68,111,149,31,64,126,4,47,76,119,154,40};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[74][row74Pos[idx]] = row74Vals[idx];
    }

    // ========== 第75行 ==========
    uint8_t row75Vals[] = {14,2,63,26,41,12,17,32,58,37,38,23,55,22,46,15};
    int row75Pos[] = {79,121,159,45,69,112,150,32,65,127,5,48,77,120,155,41};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[75][row75Pos[idx]] = row75Vals[idx];
    }

    // ========== 第76行 ==========
    uint8_t row76Vals[] = {53,15,31,61,26,22,14,2,63,26,41,12,17,32,58,37};
    int row76Pos[] = {80,122,160,46,70,113,151,33,66,128,6,49,78,121,156,42};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[76][row76Pos[idx]] = row76Vals[idx];
    }

    // ========== 第77行 ==========
    uint8_t row77Vals[] = {38,23,55,22,46,15,53,15,31,61,26,22,14,2,63,26};
    int row77Pos[] = {81,123,0,47,71,114,152,34,67,129,7,50,79,122,157,43};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[77][row77Pos[idx]] = row77Vals[idx];
    }

    // ========== 第78行 ==========
    uint8_t row78Vals[] = {41,12,17,32,58,37,38,23,55,22,46,15,53,15,31,61};
    int row78Pos[] = {82,124,1,48,72,115,153,35,68,130,8,51,80,123,158,44};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[78][row78Pos[idx]] = row78Vals[idx];
    }

    // ========== 第79行 ==========
    uint8_t row79Vals[] = {26,22,14,2,63,26,41,12,17,32,58,37,38,23,55,22};
    int row79Pos[] = {83,125,2,49,73,116,154,36,69,131,9,52,81,124,159,45};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[79][row79Pos[idx]] = row79Vals[idx];
    }

    // ========== 第80行 ==========
    uint8_t row80Vals[] = {46,15,53,15,31,61,26,22,14,2,63,26,41,12,17,32};
    int row80Pos[] = {84,126,3,50,74,117,155,37,70,132,10,53,82,125,160,46};
    for (int idx = 0; idx < 16; ++idx) {
        m_ldpcH[80][row80Pos[idx]] = row80Vals[idx];
    }

    //emit receiveLog("LDPC校验矩阵（81×162）初始化完成，符合GB/T 39414.5-2024附录A");
}

QByteArray Receiver::ldpcDecode(const QByteArray &encodedData)
{
    if (encodedData.size() != 162) {
        //emit receiveLog("LDPC编码数据长度错误（需162字节）");
        return QByteArray();
    }

    QVector<uint8_t> symbols;
    for (char c : encodedData) symbols.append(static_cast<uint8_t>(c));

    const int maxIter = 50;
    const int nSymbol = 162;
    const int kSymbol = 81;
    const int nFFT = 64;
    const int truncateLen = 16;

    // 初始化LLR
    QVector<QVector<float>> llr(nSymbol, QVector<float>(nFFT, 10.0f));
    for (int i = 0; i < nSymbol; ++i) {
        llr[i][symbols[i]] = -10.0f;
    }

    for (int iter = 0; iter < maxIter; ++iter) {
        QVector<QVector<QVector<float>>> v2c(nSymbol, QVector<QVector<float>>(81, QVector<float>(truncateLen, 10.0f)));
        QVector<QVector<QVector<float>>> c2v(81, QVector<QVector<float>>(nSymbol, QVector<float>(truncateLen, 10.0f)));

        // 变量节点更新
        for (int j = 0; j < nSymbol; ++j) {
            QVector<float> combinedLLR(nFFT, 10.0f);
            for (int i = 0; i < 81; ++i) {
                if (m_ldpcH[i][j] == 0) continue;
                QVector<float> c2vLLR = expandTruncatedLLR(c2v[i][j], truncateLen, nFFT);
                for (int f = 0; f < nFFT; ++f) {
                    combinedLLR[f] += c2vLLR[f];
                }
            }
            for (int f = 0; f < nFFT; ++f) {
                combinedLLR[f] += llr[j][f];
            }

            for (int i = 0; i < 81; ++i) {
                if (m_ldpcH[i][j] == 0) continue;
                QVector<float> truncated = truncateLLR(combinedLLR, truncateLen);
                v2c[j][i] = multiplyLLRByGF(truncated, gf26Inv(m_ldpcH[i][j]), nFFT);
            }
        }

        // 校验节点更新
        for (int i = 0; i < 81; ++i) {
            QVector<QVector<float>> relatedV2C;
            QVector<int> relatedJ;
            for (int j = 0; j < nSymbol; ++j) {
                if (m_ldpcH[i][j] != 0) {
                    relatedV2C.append(v2c[j][i]);
                    relatedJ.append(j);
                }
            }

            for (int idx = 0; idx < relatedJ.size(); ++idx) {
                int j = relatedJ[idx];
                QVector<float> sumLLR(nFFT, 10.0f);
                for (int k = 0; k < relatedJ.size(); ++k) {
                    if (k == idx) continue;
                    QVector<float> v2cLLR = expandTruncatedLLR(relatedV2C[k], truncateLen, nFFT);
                    for (int f = 0; f < nFFT; ++f) {
                        sumLLR[f] = std::min(sumLLR[f], v2cLLR[f]);
                    }
                }
                c2v[i][j] = truncateLLR(multiplyLLRByGF(sumLLR, m_ldpcH[i][j], nFFT), truncateLen);
            }
        }

        // 硬判决+校验和验证
        QVector<uint8_t> hardDecision(nSymbol);
        bool checkPass = true;
        for (int j = 0; j < nSymbol; ++j) {
            QVector<float> finalLLR(nFFT, 10.0f);
            for (int i = 0; i < 81; ++i) {
                if (m_ldpcH[i][j] == 0) continue;
                QVector<float> c2vLLR = expandTruncatedLLR(c2v[i][j], truncateLen, nFFT);
                for (int f = 0; f < nFFT; ++f) {
                    finalLLR[f] += c2vLLR[f];
                }
            }
            for (int f = 0; f < nFFT; ++f) {
                finalLLR[f] += llr[j][f];
            }
            hardDecision[j] = static_cast<uint8_t>(std::min_element(finalLLR.begin(), finalLLR.end()) - finalLLR.begin());
        }

        // 校验和验证（GF(2^6)域）
        for (int i = 0; i < 81; ++i) {
            uint8_t sum = 0;
            for (int j = 0; j < nSymbol; ++j) {
                if (m_ldpcH[i][j] != 0) {
                    sum = gf26Add(sum, gf26Mul(hardDecision[j], m_ldpcH[i][j]));
                }
            }
            if (sum != 0) {
                checkPass = false;
                break;
            }
        }

        if (checkPass) {
            QVector<uint8_t> infoSymbols;
            for (int i = 0; i < kSymbol; ++i) {
                infoSymbols.append(hardDecision[i]);
            }
            return symbolsToBits(infoSymbols);
        }
    }

    //emit receiveLog("LDPC译码迭代次数耗尽，失败");
    return QByteArray();
}

// 第一步：工具函数（字节数组 → 比特流，高位在前，符合北斗规范）
QVector<bool> Receiver::byteArrayToBitStream(const QByteArray &byteArray)
{
    QVector<bool> bitStream;
    bitStream.reserve(byteArray.size() * 8);

    for (char byte : byteArray) {
        uint8_t uByte = static_cast<uint8_t>(byte);
        // 高位在前（北斗B-CNAV3规范：字节的第7位是最高位，第0位是最低位）
        for (int bitIdx = 7; bitIdx >= 0; --bitIdx) {
            bitStream.append((uByte >> bitIdx) & 0x01);
        }
    }
    return bitStream;
}

// 第二步：工具函数（比特流 → 字节数组，高位在前，补零对齐）
QByteArray Receiver::bitStreamToByteArray(const QVector<bool> &bitStream)
{
    QByteArray byteArray;
    int byteCount = (bitStream.size() + 7) / 8; // 向上取整

    for (int i = 0; i < byteCount; ++i) {
        uint8_t byte = 0;
        for (int bitIdx = 0; bitIdx < 8; ++bitIdx) {
            int globalBitIdx = i * 8 + bitIdx;
            if (globalBitIdx >= bitStream.size()) {
                break; // 不足8位补0
            }
            byte = (byte << 1) | (bitStream[globalBitIdx] ? 1 : 0);
        }
        byteArray.append(static_cast<char>(byte));
    }
    return byteArray;
}

// 第三步：核心比特级同步头查找
bool Receiver::findSyncHeader(QByteArray &buffer)
{
    // 同步头0xEB90的16比特序列（高位在前）
    // 0xEB = 11101011，0x90 = 10010000
    const QVector<bool> syncHeaderBits = {
        1,1,1,0,1,0,1,1, // 0xEB (第0-7位)
        1,0,0,1,0,0,0,0  // 0x90 (第8-15位)
    };
    const int syncHeaderLen = syncHeaderBits.size(); // 16位

    // 空缓存/长度不足直接返回
    if (buffer.isEmpty() || buffer.size() * 8 < syncHeaderLen) {
        emit receiveLog(QString("【同步头】缓存比特数不足（需16位，当前%1位），跳过匹配").arg(buffer.size()*8));
        return false;
    }

    // 1. 字节数组转连续比特流
    QVector<bool> bitStream = byteArrayToBitStream(buffer);
    emit receiveLog(QString("【同步头】转换为比特流，长度：%1位").arg(bitStream.size()));

    // 2. 滑动匹配16比特同步头（比特级）
    int matchPos = -1;
    for (int i = 0; i <= bitStream.size() - syncHeaderLen; ++i) {
        bool match = true;
        for (int j = 0; j < syncHeaderLen; ++j) {
            if (bitStream[i + j] != syncHeaderBits[j]) {
                match = false;
                break;
            }
        }
        if (match) {
            matchPos = i;
            break;
        }
    }

    // 3. 未找到同步头
    if (matchPos == -1) {
        emit receiveLog(QString("【同步头】比特级匹配失败，未找到0xEB90（16位序列），当前缓存比特数：%1").arg(bitStream.size()));
        // 保留最后15比特（避免同步头跨缓存，下次拼接后可匹配）
        if (bitStream.size() > 15) {
            bitStream = bitStream.mid(bitStream.size() - 15);
        }
        buffer = bitStreamToByteArray(bitStream); // 转回字节缓存
        return false;
    }

    // 4. 找到同步头：截断到同步头起始位置
    emit receiveLog(QString("【同步头】成功找到！起始比特位置：%1，清理前%1位无效数据").arg(matchPos));
    bitStream = bitStream.mid(matchPos); // 截断到同步头起始位
    buffer = bitStreamToByteArray(bitStream); // 重新转换为字节数组（保证后续字节对齐）

    return true;
}

bool Receiver::parseFrameHeader(const QByteArray &frame, uint8_t &prn, uint8_t &msgType)
{
    if (frame.size() < 4) {
        //emit receiveLog("帧长度不足4字节，无法解析PRN和信息类型");
        return false;
    }

    uint8_t byte1 = static_cast<uint8_t>(frame[0]);
    uint8_t byte2 = static_cast<uint8_t>(frame[1]);

    // 解析PRN（6bit，规范8.1）
    prn = (byte1 >> 2) & 0x3F;
    // 解析信息类型（6bit，规范8.2）
    uint8_t msgTypeHigh2 = (byte1 & 0x03);
    uint8_t msgTypeLow4 = (byte2 >> 4);
    msgType = (msgTypeHigh2 << 4) | msgTypeLow4;

    if (msgType != 10 && msgType != 30 && msgType != 40) {
        //emit receiveLog(QString("无效信息类型：%1（规范有效值：10/30/40）").arg(msgType));
        return false;
    }

    return true;
}

uint32_t Receiver::calculateCRC24Q(const QByteArray &data)
{
    const uint32_t poly = 0x1864CFB; // 规范7.1.2
    uint32_t crc = 0x000000;

    for (int i = 0; i < data.size(); ++i) {
        uint8_t byte = static_cast<uint8_t>(data[i]);
        crc ^= static_cast<uint32_t>(byte) << 16;

        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x800000) {
                crc = (crc << 1) ^ poly;
            } else {
                crc <<= 1;
            }
            crc &= 0xFFFFFF;
        }
    }
    return crc & 0xFFFFFF;
}

bool Receiver::verifyCRC(const QByteArray &rawFrame)
{
    QByteArray crcData = rawFrame.left(58);
    uint32_t calculatedCrc = calculateCRC24Q(crcData);

    QByteArray frameCrcBytes = rawFrame.right(3);
    uint32_t frameCrc = 0;
    for (int i = 0; i < 3; ++i) {
        frameCrc = (frameCrc << 8) | static_cast<uint8_t>(frameCrcBytes[i]);
    }

    bool isMatch = (calculatedCrc == frameCrc);
    if (!isMatch) {
        //emit receiveLog(QString("CRC校验失败：计算值=0x%1，帧中值=0x%2").arg(calculatedCrc, 6, 16, QChar('0')).arg(frameCrc, 6, 16, QChar('0')));
    }
    return isMatch;
}

QByteArray Receiver::symbolsToBits(const QVector<uint8_t> &symbols)
{
    QByteArray bits;
    for (uint8_t sym : symbols) {
        for (int i = 5; i >= 0; --i) {
            bits.append((sym >> i) & 0x01 ? 0x01 : 0x00);
        }
    }

    QByteArray byteData;
    for (int i = 0; i < bits.size(); i += 8) {
        uint8_t byte = 0;
        for (int j = 0; j < 8 && (i+j) < bits.size(); ++j) {
            byte = (byte << 1) | (bits[i+j] & 0x01);
        }
        byteData.append(static_cast<char>(byte));
    }
    return byteData;
}

uint8_t Receiver::gf26Add(uint8_t a, uint8_t b)
{
    return a ^ b; // 规范附录A：GF(2^6)加法=异或
}

uint8_t Receiver::gf26Mul(uint8_t a, uint8_t b)
{
    uint8_t result = 0;
    const uint8_t poly = 0x03; // 规范7.2.2：本原多项式x^6+x+1
    while (a && b) {
        if (b & 1) result ^= a;
        bool carry = (a & 0x20) != 0;
        a <<= 1;
        if (carry) a ^= poly;
        b >>= 1;
    }
    return result;
}

uint8_t Receiver::gf26Inv(uint8_t a)
{
    return m_gf26InvTable[a % 64];
}

QVector<float> Receiver::truncateLLR(const QVector<float> &llr, int n)
{
    QVector<std::pair<float, int>> pairs;
    for (int i = 0; i < llr.size(); ++i) {
        pairs.append({llr[i], i});
    }
    // 替换auto为显式的std::pair<float, int>类型，适配C++11
    std::sort(pairs.begin(), pairs.end(), [](const std::pair<float, int>& a, const std::pair<float, int>& b) {
        return a.first < b.first;
    });

    QVector<float> truncated(n, 10.0f);
    for (int i = 0; i < n && i < pairs.size(); ++i) {
        truncated[i] = pairs[i].first;
    }
    return truncated;
}

QVector<float> Receiver::expandTruncatedLLR(const QVector<float> &truncated, int n, int fullSize)
{
    QVector<float> full(fullSize, 10.0f);
    for (int i = 0; i < n && i < fullSize; ++i) {
        full[i] = truncated[i];
    }
    return full;
}

QVector<float> Receiver::multiplyLLRByGF(const QVector<float> &llr, uint8_t gfVal, int fullSize)
{
    QVector<float> result(fullSize, 10.0f);
    for (int f = 0; f < fullSize; ++f) {
        uint8_t newF = gf26Mul(static_cast<uint8_t>(f), gfVal);
        result[newF] = std::min(result[newF], llr[f]);
    }
    return result;
}
