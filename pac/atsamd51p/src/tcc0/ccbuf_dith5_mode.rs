#[doc = "Register `CCBUF_DITH5_MODE[%s]` reader"]
pub type R = crate::R<CCBUF_DITH5_MODE_SPEC>;
#[doc = "Register `CCBUF_DITH5_MODE[%s]` writer"]
pub type W = crate::W<CCBUF_DITH5_MODE_SPEC>;
#[doc = "Field `DITHERBUF` reader - Dithering Buffer Cycle Number"]
pub type DITHERBUF_R = crate::FieldReader;
#[doc = "Field `DITHERBUF` writer - Dithering Buffer Cycle Number"]
pub type DITHERBUF_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 5, O>;
#[doc = "Field `CCBUF` reader - Channel Compare/Capture Buffer Value"]
pub type CCBUF_R = crate::FieldReader<u32>;
#[doc = "Field `CCBUF` writer - Channel Compare/Capture Buffer Value"]
pub type CCBUF_W<'a, REG, const O: u8> = crate::FieldWriter<'a, REG, 19, O, u32>;
impl R {
    #[doc = "Bits 0:4 - Dithering Buffer Cycle Number"]
    #[inline(always)]
    pub fn ditherbuf(&self) -> DITHERBUF_R {
        DITHERBUF_R::new((self.bits & 0x1f) as u8)
    }
    #[doc = "Bits 5:23 - Channel Compare/Capture Buffer Value"]
    #[inline(always)]
    pub fn ccbuf(&self) -> CCBUF_R {
        CCBUF_R::new((self.bits >> 5) & 0x0007_ffff)
    }
}
impl W {
    #[doc = "Bits 0:4 - Dithering Buffer Cycle Number"]
    #[inline(always)]
    #[must_use]
    pub fn ditherbuf(&mut self) -> DITHERBUF_W<CCBUF_DITH5_MODE_SPEC, 0> {
        DITHERBUF_W::new(self)
    }
    #[doc = "Bits 5:23 - Channel Compare/Capture Buffer Value"]
    #[inline(always)]
    #[must_use]
    pub fn ccbuf(&mut self) -> CCBUF_W<CCBUF_DITH5_MODE_SPEC, 5> {
        CCBUF_W::new(self)
    }
    #[doc = r" Writes raw bits to the register."]
    #[doc = r""]
    #[doc = r" # Safety"]
    #[doc = r""]
    #[doc = r" Passing incorrect value can cause undefined behaviour. See reference manual"]
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: u32) -> &mut Self {
        self.bits = bits;
        self
    }
}
#[doc = "Compare and Capture Buffer\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`ccbuf_dith5_mode::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`ccbuf_dith5_mode::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct CCBUF_DITH5_MODE_SPEC;
impl crate::RegisterSpec for CCBUF_DITH5_MODE_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ccbuf_dith5_mode::R`](R) reader structure"]
impl crate::Readable for CCBUF_DITH5_MODE_SPEC {}
#[doc = "`write(|w| ..)` method takes [`ccbuf_dith5_mode::W`](W) writer structure"]
impl crate::Writable for CCBUF_DITH5_MODE_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets CCBUF_DITH5_MODE[%s]
to value 0"]
impl crate::Resettable for CCBUF_DITH5_MODE_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
