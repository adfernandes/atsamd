#[doc = "Register `INTENSET` reader"]
pub type R = crate::R<INTENSET_SPEC>;
#[doc = "Register `INTENSET` writer"]
pub type W = crate::W<INTENSET_SPEC>;
#[doc = "Field `OVF` reader - Overflow Interrupt Enable"]
pub type OVF_R = crate::BitReader;
#[doc = "Field `OVF` writer - Overflow Interrupt Enable"]
pub type OVF_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `TRG` reader - Retrigger Interrupt Enable"]
pub type TRG_R = crate::BitReader;
#[doc = "Field `TRG` writer - Retrigger Interrupt Enable"]
pub type TRG_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `CNT` reader - Counter Interrupt Enable"]
pub type CNT_R = crate::BitReader;
#[doc = "Field `CNT` writer - Counter Interrupt Enable"]
pub type CNT_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `ERR` reader - Error Interrupt Enable"]
pub type ERR_R = crate::BitReader;
#[doc = "Field `ERR` writer - Error Interrupt Enable"]
pub type ERR_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `UFS` reader - Non-Recoverable Update Fault Interrupt Enable"]
pub type UFS_R = crate::BitReader;
#[doc = "Field `UFS` writer - Non-Recoverable Update Fault Interrupt Enable"]
pub type UFS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `DFS` reader - Non-Recoverable Debug Fault Interrupt Enable"]
pub type DFS_R = crate::BitReader;
#[doc = "Field `DFS` writer - Non-Recoverable Debug Fault Interrupt Enable"]
pub type DFS_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `FAULTA` reader - Recoverable Fault A Interrupt Enable"]
pub type FAULTA_R = crate::BitReader;
#[doc = "Field `FAULTA` writer - Recoverable Fault A Interrupt Enable"]
pub type FAULTA_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `FAULTB` reader - Recoverable Fault B Interrupt Enable"]
pub type FAULTB_R = crate::BitReader;
#[doc = "Field `FAULTB` writer - Recoverable Fault B Interrupt Enable"]
pub type FAULTB_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `FAULT0` reader - Non-Recoverable Fault 0 Interrupt Enable"]
pub type FAULT0_R = crate::BitReader;
#[doc = "Field `FAULT0` writer - Non-Recoverable Fault 0 Interrupt Enable"]
pub type FAULT0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `FAULT1` reader - Non-Recoverable Fault 1 Interrupt Enable"]
pub type FAULT1_R = crate::BitReader;
#[doc = "Field `FAULT1` writer - Non-Recoverable Fault 1 Interrupt Enable"]
pub type FAULT1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC0` reader - Match or Capture Channel 0 Interrupt Enable"]
pub type MC0_R = crate::BitReader;
#[doc = "Field `MC0` writer - Match or Capture Channel 0 Interrupt Enable"]
pub type MC0_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC1` reader - Match or Capture Channel 1 Interrupt Enable"]
pub type MC1_R = crate::BitReader;
#[doc = "Field `MC1` writer - Match or Capture Channel 1 Interrupt Enable"]
pub type MC1_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC2` reader - Match or Capture Channel 2 Interrupt Enable"]
pub type MC2_R = crate::BitReader;
#[doc = "Field `MC2` writer - Match or Capture Channel 2 Interrupt Enable"]
pub type MC2_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC3` reader - Match or Capture Channel 3 Interrupt Enable"]
pub type MC3_R = crate::BitReader;
#[doc = "Field `MC3` writer - Match or Capture Channel 3 Interrupt Enable"]
pub type MC3_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC4` reader - Match or Capture Channel 4 Interrupt Enable"]
pub type MC4_R = crate::BitReader;
#[doc = "Field `MC4` writer - Match or Capture Channel 4 Interrupt Enable"]
pub type MC4_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
#[doc = "Field `MC5` reader - Match or Capture Channel 5 Interrupt Enable"]
pub type MC5_R = crate::BitReader;
#[doc = "Field `MC5` writer - Match or Capture Channel 5 Interrupt Enable"]
pub type MC5_W<'a, REG, const O: u8> = crate::BitWriter<'a, REG, O>;
impl R {
    #[doc = "Bit 0 - Overflow Interrupt Enable"]
    #[inline(always)]
    pub fn ovf(&self) -> OVF_R {
        OVF_R::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1 - Retrigger Interrupt Enable"]
    #[inline(always)]
    pub fn trg(&self) -> TRG_R {
        TRG_R::new(((self.bits >> 1) & 1) != 0)
    }
    #[doc = "Bit 2 - Counter Interrupt Enable"]
    #[inline(always)]
    pub fn cnt(&self) -> CNT_R {
        CNT_R::new(((self.bits >> 2) & 1) != 0)
    }
    #[doc = "Bit 3 - Error Interrupt Enable"]
    #[inline(always)]
    pub fn err(&self) -> ERR_R {
        ERR_R::new(((self.bits >> 3) & 1) != 0)
    }
    #[doc = "Bit 10 - Non-Recoverable Update Fault Interrupt Enable"]
    #[inline(always)]
    pub fn ufs(&self) -> UFS_R {
        UFS_R::new(((self.bits >> 10) & 1) != 0)
    }
    #[doc = "Bit 11 - Non-Recoverable Debug Fault Interrupt Enable"]
    #[inline(always)]
    pub fn dfs(&self) -> DFS_R {
        DFS_R::new(((self.bits >> 11) & 1) != 0)
    }
    #[doc = "Bit 12 - Recoverable Fault A Interrupt Enable"]
    #[inline(always)]
    pub fn faulta(&self) -> FAULTA_R {
        FAULTA_R::new(((self.bits >> 12) & 1) != 0)
    }
    #[doc = "Bit 13 - Recoverable Fault B Interrupt Enable"]
    #[inline(always)]
    pub fn faultb(&self) -> FAULTB_R {
        FAULTB_R::new(((self.bits >> 13) & 1) != 0)
    }
    #[doc = "Bit 14 - Non-Recoverable Fault 0 Interrupt Enable"]
    #[inline(always)]
    pub fn fault0(&self) -> FAULT0_R {
        FAULT0_R::new(((self.bits >> 14) & 1) != 0)
    }
    #[doc = "Bit 15 - Non-Recoverable Fault 1 Interrupt Enable"]
    #[inline(always)]
    pub fn fault1(&self) -> FAULT1_R {
        FAULT1_R::new(((self.bits >> 15) & 1) != 0)
    }
    #[doc = "Bit 16 - Match or Capture Channel 0 Interrupt Enable"]
    #[inline(always)]
    pub fn mc0(&self) -> MC0_R {
        MC0_R::new(((self.bits >> 16) & 1) != 0)
    }
    #[doc = "Bit 17 - Match or Capture Channel 1 Interrupt Enable"]
    #[inline(always)]
    pub fn mc1(&self) -> MC1_R {
        MC1_R::new(((self.bits >> 17) & 1) != 0)
    }
    #[doc = "Bit 18 - Match or Capture Channel 2 Interrupt Enable"]
    #[inline(always)]
    pub fn mc2(&self) -> MC2_R {
        MC2_R::new(((self.bits >> 18) & 1) != 0)
    }
    #[doc = "Bit 19 - Match or Capture Channel 3 Interrupt Enable"]
    #[inline(always)]
    pub fn mc3(&self) -> MC3_R {
        MC3_R::new(((self.bits >> 19) & 1) != 0)
    }
    #[doc = "Bit 20 - Match or Capture Channel 4 Interrupt Enable"]
    #[inline(always)]
    pub fn mc4(&self) -> MC4_R {
        MC4_R::new(((self.bits >> 20) & 1) != 0)
    }
    #[doc = "Bit 21 - Match or Capture Channel 5 Interrupt Enable"]
    #[inline(always)]
    pub fn mc5(&self) -> MC5_R {
        MC5_R::new(((self.bits >> 21) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0 - Overflow Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn ovf(&mut self) -> OVF_W<INTENSET_SPEC, 0> {
        OVF_W::new(self)
    }
    #[doc = "Bit 1 - Retrigger Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn trg(&mut self) -> TRG_W<INTENSET_SPEC, 1> {
        TRG_W::new(self)
    }
    #[doc = "Bit 2 - Counter Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn cnt(&mut self) -> CNT_W<INTENSET_SPEC, 2> {
        CNT_W::new(self)
    }
    #[doc = "Bit 3 - Error Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn err(&mut self) -> ERR_W<INTENSET_SPEC, 3> {
        ERR_W::new(self)
    }
    #[doc = "Bit 10 - Non-Recoverable Update Fault Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn ufs(&mut self) -> UFS_W<INTENSET_SPEC, 10> {
        UFS_W::new(self)
    }
    #[doc = "Bit 11 - Non-Recoverable Debug Fault Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn dfs(&mut self) -> DFS_W<INTENSET_SPEC, 11> {
        DFS_W::new(self)
    }
    #[doc = "Bit 12 - Recoverable Fault A Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn faulta(&mut self) -> FAULTA_W<INTENSET_SPEC, 12> {
        FAULTA_W::new(self)
    }
    #[doc = "Bit 13 - Recoverable Fault B Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn faultb(&mut self) -> FAULTB_W<INTENSET_SPEC, 13> {
        FAULTB_W::new(self)
    }
    #[doc = "Bit 14 - Non-Recoverable Fault 0 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn fault0(&mut self) -> FAULT0_W<INTENSET_SPEC, 14> {
        FAULT0_W::new(self)
    }
    #[doc = "Bit 15 - Non-Recoverable Fault 1 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn fault1(&mut self) -> FAULT1_W<INTENSET_SPEC, 15> {
        FAULT1_W::new(self)
    }
    #[doc = "Bit 16 - Match or Capture Channel 0 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc0(&mut self) -> MC0_W<INTENSET_SPEC, 16> {
        MC0_W::new(self)
    }
    #[doc = "Bit 17 - Match or Capture Channel 1 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc1(&mut self) -> MC1_W<INTENSET_SPEC, 17> {
        MC1_W::new(self)
    }
    #[doc = "Bit 18 - Match or Capture Channel 2 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc2(&mut self) -> MC2_W<INTENSET_SPEC, 18> {
        MC2_W::new(self)
    }
    #[doc = "Bit 19 - Match or Capture Channel 3 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc3(&mut self) -> MC3_W<INTENSET_SPEC, 19> {
        MC3_W::new(self)
    }
    #[doc = "Bit 20 - Match or Capture Channel 4 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc4(&mut self) -> MC4_W<INTENSET_SPEC, 20> {
        MC4_W::new(self)
    }
    #[doc = "Bit 21 - Match or Capture Channel 5 Interrupt Enable"]
    #[inline(always)]
    #[must_use]
    pub fn mc5(&mut self) -> MC5_W<INTENSET_SPEC, 21> {
        MC5_W::new(self)
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
#[doc = "Interrupt Enable Set\n\nYou can [`read`](crate::generic::Reg::read) this register and get [`intenset::R`](R).  You can [`reset`](crate::generic::Reg::reset), [`write`](crate::generic::Reg::write), [`write_with_zero`](crate::generic::Reg::write_with_zero) this register using [`intenset::W`](W). You can also [`modify`](crate::generic::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct INTENSET_SPEC;
impl crate::RegisterSpec for INTENSET_SPEC {
    type Ux = u32;
}
#[doc = "`read()` method returns [`intenset::R`](R) reader structure"]
impl crate::Readable for INTENSET_SPEC {}
#[doc = "`write(|w| ..)` method takes [`intenset::W`](W) writer structure"]
impl crate::Writable for INTENSET_SPEC {
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux = 0;
}
#[doc = "`reset()` method sets INTENSET to value 0"]
impl crate::Resettable for INTENSET_SPEC {
    const RESET_VALUE: Self::Ux = 0;
}
