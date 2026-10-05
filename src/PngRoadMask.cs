using System;
using System.IO;
using System.IO.Compression;

namespace T3taAutopilot
{
    /// <summary>
    /// Streams a (large) 8-bit RGB/RGBA PNG row by row and downsamples it to a
    /// 1-byte-per-cell mask, so an 8192^2 splatmap can be read with a few MB
    /// of memory instead of a 268 MB Color32[] (plus a Texture2D).
    /// Output row 0 = image BOTTOM row = world south (RWG splatmaps have
    /// north at the top; checked against prefabs.xml road-side parts).
    /// Each output cell holds the highest class any of its source pixels got.
    /// No engine dependencies (shared with tools/sim).
    /// </summary>
    internal static class PngRoadMask
    {
        /// <summary>Pixel -> class (0 = nothing, higher wins within a cell).</summary>
        public delegate byte Classify(byte r, byte g, byte b, byte a);

        public static byte[] Load(string file, int factor, Classify classify, out int outW, out int outH)
        {
            outW = outH = 0;
            using (var fs = new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.Read, 1 << 16))
            {
                var sig = new byte[8];
                if (fs.Read(sig, 0, 8) != 8 || sig[0] != 0x89 || sig[1] != (byte)'P')
                    throw new InvalidDataException("not a PNG: " + file);

                // IHDR
                int len = ReadInt(fs);
                string type = ReadType(fs);
                if (type != "IHDR") throw new InvalidDataException("IHDR expected");
                var ihdr = new byte[len];
                ReadExact(fs, ihdr, len);
                ReadInt(fs); // crc
                int w = (ihdr[0] << 24) | (ihdr[1] << 16) | (ihdr[2] << 8) | ihdr[3];
                int h = (ihdr[4] << 24) | (ihdr[5] << 16) | (ihdr[6] << 8) | ihdr[7];
                int depth = ihdr[8], color = ihdr[9], interlace = ihdr[12];
                if (depth != 8 || (color != 6 && color != 2) || interlace != 0)
                    throw new NotSupportedException("PNG must be 8-bit RGB/RGBA, non-interlaced (depth " +
                        depth + ", color " + color + ", interlace " + interlace + ")");
                int bpp = color == 6 ? 4 : 3;
                int stride = w * bpp;

                outW = (w + factor - 1) / factor;
                outH = (h + factor - 1) / factor;
                var mask = new byte[outW * outH];

                using (var idat = new IdatStream(fs))
                {
                    // zlib header (2 bytes) then raw deflate
                    idat.ReadByte();
                    idat.ReadByte();
                    using (var z = new DeflateStream(idat, CompressionMode.Decompress))
                    {
                        var prev = new byte[stride];
                        var cur = new byte[stride];
                        for (int y = 0; y < h; y++)
                        {
                            int filter = z.ReadByte();
                            if (filter < 0) throw new EndOfStreamException("PNG data truncated at row " + y);
                            ReadExact(z, cur, stride);
                            Unfilter(filter, cur, prev, bpp);

                            int orow = ((h - 1 - y) / factor) * outW;
                            for (int x = 0, p = 0; x < w; x++, p += bpp)
                            {
                                byte c = classify(cur[p], cur[p + 1], cur[p + 2], bpp == 4 ? cur[p + 3] : (byte)255);
                                if (c > mask[orow + x / factor])
                                {
                                    mask[orow + x / factor] = c;
                                }
                            }
                            var t = prev; prev = cur; cur = t;
                        }
                    }
                }
                return mask;
            }
        }

        /// <summary>Image width / height from the IHDR chunk.</summary>
        public static void ReadSize(string file, out int w, out int h)
        {
            using (var fs = new FileStream(file, FileMode.Open, FileAccess.Read, FileShare.Read))
            {
                var hdr = new byte[24];
                ReadExact(fs, hdr, 24);   // signature(8) + length(4) + "IHDR"(4) + width(4) + height(4)
                if (hdr[0] != 0x89 || hdr[1] != (byte)'P' || hdr[12] != (byte)'I' || hdr[13] != (byte)'H')
                    throw new InvalidDataException("not a PNG: " + file);
                w = (hdr[16] << 24) | (hdr[17] << 16) | (hdr[18] << 8) | hdr[19];
                h = (hdr[20] << 24) | (hdr[21] << 16) | (hdr[22] << 8) | hdr[23];
            }
        }

        static void Unfilter(int filter, byte[] cur, byte[] prev, int bpp)
        {
            int n = cur.Length;
            switch (filter)
            {
                case 0:
                    break;
                case 1:
                    for (int i = bpp; i < n; i++) cur[i] = (byte)(cur[i] + cur[i - bpp]);
                    break;
                case 2:
                    for (int i = 0; i < n; i++) cur[i] = (byte)(cur[i] + prev[i]);
                    break;
                case 3:
                    for (int i = 0; i < n; i++)
                    {
                        int left = i >= bpp ? cur[i - bpp] : 0;
                        cur[i] = (byte)(cur[i] + ((left + prev[i]) >> 1));
                    }
                    break;
                case 4:
                    for (int i = 0; i < n; i++)
                    {
                        int a = i >= bpp ? cur[i - bpp] : 0;
                        int b = prev[i];
                        int c = i >= bpp ? prev[i - bpp] : 0;
                        int pa = Math.Abs(b - c), pb = Math.Abs(a - c), pc = Math.Abs(a + b - 2 * c);
                        int pred = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);
                        cur[i] = (byte)(cur[i] + pred);
                    }
                    break;
                default:
                    throw new InvalidDataException("bad PNG filter " + filter);
            }
        }

        static int ReadInt(Stream s)
        {
            int b0 = s.ReadByte(), b1 = s.ReadByte(), b2 = s.ReadByte(), b3 = s.ReadByte();
            if (b3 < 0) throw new EndOfStreamException();
            return (b0 << 24) | (b1 << 16) | (b2 << 8) | b3;
        }

        static string ReadType(Stream s)
        {
            var t = new byte[4];
            ReadExact(s, t, 4);
            return System.Text.Encoding.ASCII.GetString(t);
        }

        static void ReadExact(Stream s, byte[] buf, int count)
        {
            int off = 0;
            while (off < count)
            {
                int r = s.Read(buf, off, count - off);
                if (r <= 0) throw new EndOfStreamException();
                off += r;
            }
        }

        /// <summary>Concatenated payload of consecutive IDAT chunks.</summary>
        sealed class IdatStream : Stream
        {
            readonly Stream src;
            int left;          // bytes left in the current IDAT chunk
            bool done;

            public IdatStream(Stream s)
            {
                src = s;
            }

            bool NextChunk()
            {
                for (;;)
                {
                    if (src.Position >= src.Length) return false;
                    int len = ReadInt(src);
                    string type = ReadType(src);
                    if (type == "IDAT")
                    {
                        left = len;
                        return true;
                    }
                    if (type == "IEND") return false;
                    src.Seek(len + 4, SeekOrigin.Current);   // skip data + crc
                }
            }

            public override int Read(byte[] buffer, int offset, int count)
            {
                if (done) return 0;
                while (left == 0)
                {
                    if (!NextChunk()) { done = true; return 0; }
                    if (left == 0) src.Seek(4, SeekOrigin.Current);   // empty IDAT: skip crc
                }
                int r = src.Read(buffer, offset, Math.Min(count, left));
                if (r <= 0) { done = true; return 0; }
                left -= r;
                if (left == 0) src.Seek(4, SeekOrigin.Current);          // crc
                return r;
            }

            public override bool CanRead => true;
            public override bool CanSeek => false;
            public override bool CanWrite => false;
            public override long Length => throw new NotSupportedException();
            public override long Position { get => throw new NotSupportedException(); set => throw new NotSupportedException(); }
            public override void Flush() { }
            public override long Seek(long offset, SeekOrigin origin) => throw new NotSupportedException();
            public override void SetLength(long value) => throw new NotSupportedException();
            public override void Write(byte[] buffer, int offset, int count) => throw new NotSupportedException();
        }
    }
}
