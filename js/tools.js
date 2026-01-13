// SHA-256 compute helper using Web Crypto API
async function sha256Hex(text) {
    const enc = new TextEncoder();
    const data = enc.encode(text);
    const hashBuffer = await crypto.subtle.digest('SHA-256', data);
    const hashArray = Array.from(new Uint8Array(hashBuffer));
    return hashArray.map(b => b.toString(16).padStart(2, '0')).join('');
}
/* * HARDWARE PRIMITIVES 
    * These functions simulate specific logic gates/circuits.
    * Integers in JS are bitwise-treated as 32-bit signed, so we use (>>> 0) to enforce unsigned behavior.
    */

// Right Rotate (Circular Shift) - Corresponds to barrel shifter
const rotr = (n, x) => (x >>> n) | (x << (32 - n));

// Right Shift - Corresponds to standard shift register
const shr = (n, x) => x >>> n;

// Choice Function: If x then y else z
const Ch = (x, y, z) => (x & y) ^ (~x & z);

// Majority Function: True if at least 2 inputs are true
const Maj = (x, y, z) => (x & y) ^ (x & z) ^ (y & z);

// Sigma functions (Transformations for the message schedule)
const sigma0 = (x) => rotr(7, x) ^ rotr(18, x) ^ shr(3, x);
const sigma1 = (x) => rotr(17, x) ^ rotr(19, x) ^ shr(10, x);

// Capital Sigma functions (Transformations for the compression function)
const SIGMA0 = (x) => rotr(2, x) ^ rotr(13, x) ^ rotr(22, x);
const SIGMA1 = (x) => rotr(6, x) ^ rotr(11, x) ^ rotr(25, x);

// K Constants (First 32 bits of fractional parts of cube roots of first 64 primes)
const K = [
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2
];
async function computeSHA256(text) {
    // 1. PRE-PROCESSING (PADDING)
    // In hardware, this usually happens in a stream. Here we simulate building the block RAM.
    
    // Convert string to array of bytes (8-bit)
    const msgBuffer = new TextEncoder().encode(text);

    
    // Total length of message in bits
    const len = msgBuffer.length * 8;

    document.getElementById('MessageBufferDebug').innerText = msgBuffer.reduce((acc, byte) => acc + byte.toString(16).padStart(2, '0') + " ", "") + `\nLength (bits): ${len}`;
    
    // Calculate padding:
    // We need to append '1', then '0's, so that length = 448 mod 512
    // Then append 64-bit length.
    
    // Create a buffer that can hold the padded message
    // k is number of zero bits. len + 1 + k = 448 mod 512
    let k = (448 - (len + 1)) % 512;
    if (k < 0) k += 512;
    
    // Total bytes = (len + 1 + k + 64) / 8
    const totalBits = len + 1 + k + 64;
    const totalBytes = totalBits / 8;
    
    const padded = new Uint8Array(totalBytes);
    padded.set(msgBuffer);
    
    // Append '1' bit (0x80 in hex byte)
    padded[msgBuffer.length] = 0x80;
    
    // The 0s are already there because new Uint8Array initializes to 0
    
    // Append 64-bit length at the end (Big Endian)
    // JS bitwise is 32-bit, so we handle high and low words.
    // We only support message lengths up to 2^32 bits for this simulation
    const lenHigh = Math.floor(len / 0x100000000); 
    const lenLow = len >>> 0;
    
    // Put length in last 8 bytes
    const view = new DataView(padded.buffer);
    view.setUint32(totalBytes - 8, lenHigh, false); // false = Big Endian
    view.setUint32(totalBytes - 4, lenLow, false);

    // Debug output for padding
    let paddingHex = "";
    for(let b of padded) {
        paddingHex += b.toString(16).padStart(2, '0') + " ";
    }
    document.getElementById('paddingDebug').innerText = `${len} bits of the original message + 1 bit for the '1' padding + ${k} bits of 0 padding + 64 bits for length = 512 bits\nRaw Bytes (with padding & length):\n${paddingHex}`;

    // 2. PARSING INTO BLOCKS
    // Break into 512-bit (64-byte) blocks. 
    // We treat these as array of sixteen 32-bit words (M[0]...M[15])
    
    // Initial Hash Values (H registers)
    let H = [
        0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
        0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19
    ];

    const blockCount = padded.length / 64;

    // 3. HASH COMPUTATION LOOP
    for (let i = 0; i < blockCount; i++) {
        
        // Create the Message Schedule Array W[0..63]
        const W = new Uint32Array(64);
        
        // 16 words from the message block
        for (let t = 0; t < 16; t++) {
            W[t] = view.getUint32((i * 64) + (t * 4), false);
        }

        // Extend the first 16 words into the remaining 48 words of W
        for (let t = 16; t < 64; t++) {
            // W[t] = sigma1(W[t-2]) + W[t-7] + sigma0(W[t-15]) + W[t-16]
            // We use (>>> 0) to emulate 32-bit unsigned wrap-around addition
            const s1 = sigma1(W[t - 2]);
            const s0 = sigma0(W[t - 15]);
            W[t] = (s1 + W[t - 7] + s0 + W[t - 16]) >>> 0;
        }

        // Initialize working variables for this block
        let a = H[0], b = H[1], c = H[2], d = H[3];
        let e = H[4], f = H[5], g = H[6], h = H[7];

        // 64 Rounds of Compression
        for (let t = 0; t < 64; t++) {
            const S1 = SIGMA1(e);
            const ch = Ch(e, f, g);
            // Temp1 = h + SIGMA1(e) + Ch(e, f, g) + K[t] + W[t]
            const temp1 = (h + S1 + ch + K[t] + W[t]) >>> 0;
            
            const S0 = SIGMA0(a);
            const maj = Maj(a, b, c);
            // Temp2 = SIGMA0(a) + Maj(a, b, c)
            const temp2 = (S0 + maj) >>> 0;

            // Register shifts (Simulating clock cycle updates)
            h = g;
            g = f;
            f = e;
            e = (d + temp1) >>> 0;
            d = c;
            c = b;
            b = a;
            a = (temp1 + temp2) >>> 0;
        }

        // Add the compressed chunk to the current hash value
        H[0] = (H[0] + a) >>> 0;
        H[1] = (H[1] + b) >>> 0;
        H[2] = (H[2] + c) >>> 0;
        H[3] = (H[3] + d) >>> 0;
        H[4] = (H[4] + e) >>> 0;
        H[5] = (H[5] + f) >>> 0;
        H[6] = (H[6] + g) >>> 0;
        H[7] = (H[7] + h) >>> 0;
    }

    // 4. FINAL OUTPUT
    // Convert the 8 H values to a hex string
    document.getElementById('hashOutput').innerText = H.map(h => h.toString(16).padStart(8, '0')).join(' ');
}
