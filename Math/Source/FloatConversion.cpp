// #include "UnityPrefix.h"
#include "FloatConversion.h"
// #include "Runtime/Utilities/RegisterRuntimeInitializeAndCleanup.h"

//static RegisterRuntimeInitializeAndCleanup sInitializePrecomputeTables(FloatToHalfConverter::InitializePrecomputeTables, NULL);

UInt16 FloatToHalfConverter::m_ExponentTable[256];
UInt8 FloatToHalfConverter::m_MantissaShift[256];

void FloatToHalfConverter::InitializePrecomputeTables(void*)
{
    for (int i = 0; i < 256; i++)
    {
        int e = i - 127;
        if (e < -24)
        {
            // Too small to represent becomes zero
            m_ExponentTable[i] = 0x0000;
            m_MantissaShift[i] = 24;
        }
        else if (e < -14)
        {
            // Small numbers become denormals
            m_ExponentTable[i] = 0x0400 >> (-14 - e);
            m_MantissaShift[i] = -1 - e;
        }
        else if (e < 16)
        {
            // Handle normalized numbers
            m_ExponentTable[i] = (15 + e) << 10;
            m_MantissaShift[i] = 13;
        }
        else if (e < 128)
        {
            // Large numbers become infinity
            m_ExponentTable[i] = 0x7C00;
            m_MantissaShift[i] = 24;
        }
        else
        {
            // Handle infinity and NaN
            m_ExponentTable[i] = 0x7C00;
            m_MantissaShift[i] = 13;
        }
    }
}

FloatToHalfConverter g_FloatToHalf;
