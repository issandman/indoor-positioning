using System;

namespace UnityStandardAssets.Characters.ThirdPerson
{
    internal class DataInputStream
    {
        private ByteArrayInputStream byteArrayInputStream;

        public DataInputStream(ByteArrayInputStream byteArrayInputStream)
        {
            this.byteArrayInputStream = byteArrayInputStream;
        }

        internal float readFloat()
        {
            throw new NotImplementedException();
        }
    }
}