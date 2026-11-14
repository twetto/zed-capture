//! JPEG on the Jetson's hardware encoder (NVJPG), through GStreamer: appsrc (GRAY8) → nvvidconv
//! (into NVMM memory, I420) → nvjpegenc → appsink, one image in and its JPEG out. NVJPG encodes
//! from NVMM I420 or NV12 only (from system memory nvjpegenc falls back to software, 29 ms an
//! image at 1280 × 720), so the JPEG has three channels, its chroma flat: "bgr8; jpeg compressed
//! bgr8", as mr-neuro-nav's nvjpg_encoder.py.

use gstreamer as gst;
use gstreamer::prelude::*;
use gstreamer_app as gst_app;

pub const FORMAT: &str = "bgr8; jpeg compressed bgr8";

/// One pipeline, for images of one size; used from one thread at a time.
pub struct Nvjpg {
    pipeline: gst::Pipeline,
    src: gst_app::AppSrc,
    sink: gst_app::AppSink,
    width: usize,
    height: usize,
}

fn element<T: IsA<gst::Element>>(pipeline: &gst::Pipeline, name: &str) -> Result<T, String> {
    pipeline
        .by_name(name)
        .ok_or(format!("no {name} in the pipeline"))?
        .downcast::<T>()
        .map_err(|_| format!("{name} is not the element expected"))
}

impl Nvjpg {
    /// The pipeline for `width × height` images at JPEG `quality`, tried on a blank image: the
    /// first JPEG takes ~150 ms, which would hold up the first frames' (a recording's first 2-4).
    pub fn new(width: usize, height: usize, quality: u8) -> Result<Nvjpg, String> {
        gst::init().map_err(|e| format!("GStreamer: {e}"))?;
        let description = format!(
            "appsrc name=src format=time caps=video/x-raw,format=GRAY8,width={width},height={height},framerate=0/1 \
             ! nvvidconv ! video/x-raw(memory:NVMM),format=I420 ! nvjpegenc quality={quality} \
             ! appsink name=sink sync=false"
        );
        let pipeline = gst::parse::launch(&description)
            .map_err(|e| format!("the NVJPG pipeline (JetPack's nvvidconv, nvjpegenc): {e}"))?
            .downcast::<gst::Pipeline>()
            .map_err(|_| "the NVJPG pipeline is not a pipeline".to_string())?;
        let src = element::<gst_app::AppSrc>(&pipeline, "src")?;
        let sink = element::<gst_app::AppSink>(&pipeline, "sink")?;
        pipeline
            .set_state(gst::State::Playing)
            .map_err(|e| format!("starting the NVJPG pipeline: {e}"))?;
        let n = Nvjpg {
            pipeline,
            src,
            sink,
            width,
            height,
        };
        n.encode(vec![0; width * height])?;
        Ok(n)
    }

    /// The JPEG of a grey image (`width × height` bytes, rows unpadded).
    pub fn encode(&self, grey: Vec<u8>) -> Result<Vec<u8>, String> {
        if grey.len() != self.width * self.height {
            return Err(format!(
                "an image of {} bytes for {}x{}",
                grey.len(),
                self.width,
                self.height
            ));
        }
        // GStreamer's GRAY8 rows are padded to 4 bytes
        let stride = (self.width + 3) & !3;
        let data = if stride == self.width {
            grey
        } else {
            let mut padded = vec![0u8; stride * self.height];
            for (row, out) in grey
                .chunks_exact(self.width)
                .zip(padded.chunks_exact_mut(stride))
            {
                out[..self.width].copy_from_slice(row);
            }
            padded
        };
        self.src
            .push_buffer(gst::Buffer::from_mut_slice(data))
            .map_err(|e| format!("nvjpegenc refused an image: {e:?}"))?;
        let sample = self
            .sink
            .try_pull_sample(gst::ClockTime::SECOND)
            .ok_or("nvjpegenc gave no JPEG within 1 s")?;
        let buffer = sample.buffer().ok_or("a JPEG sample without a buffer")?;
        let map = buffer
            .map_readable()
            .map_err(|e| format!("reading the JPEG: {e}"))?;
        Ok(map.as_slice().to_vec())
    }
}

impl Drop for Nvjpg {
    fn drop(&mut self) {
        let _ = self.src.end_of_stream();
        let _ = self.pipeline.set_state(gst::State::Null);
    }
}
